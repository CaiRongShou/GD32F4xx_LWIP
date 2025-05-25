#include "spi_flash.h"

/**********************************************************
 * 函 数 名 称：bsp_spi_init
 * 函 数 功 能：初始化SPI
 * 传 入 参 数：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
**********************************************************/
void bsp_spi_init(void)
{
    rcu_periph_clock_enable(BSP_GPIO_RCU);    // 使用A端口
    rcu_periph_clock_enable(BSP_SPI_RCU);     // 使能SPI0

    //引脚复用
    gpio_af_set(BSP_GPIO_PORT, GPIO_AF_5, BSP_SPI_SCK);
    gpio_af_set(BSP_GPIO_PORT, GPIO_AF_5, BSP_SPI_MISO);
    gpio_af_set(BSP_GPIO_PORT, GPIO_AF_5, BSP_SPI_MOSI);
    //引脚模式
    gpio_mode_set(BSP_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BSP_SPI_SCK);
    gpio_mode_set(BSP_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BSP_SPI_MISO);
    gpio_mode_set(BSP_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BSP_SPI_MOSI);
    //输出模式
    gpio_output_options_set(BSP_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BSP_SPI_SCK);
    gpio_output_options_set(BSP_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BSP_SPI_MISO);
    gpio_output_options_set(BSP_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BSP_SPI_MOSI);

    //开启CS引脚时钟
    rcu_periph_clock_enable(BSP_SPI_NSS_RCU);
    //配置CS引脚模式
    gpio_mode_set(BSP_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BSP_SPI_NSS);
    //配置CS输出模式
    gpio_output_options_set(BSP_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BSP_SPI_NSS);
    //GD25Q32不选中
    gpio_bit_write(BSP_GPIO_PORT, BSP_SPI_NSS, SET);

    //SPI参数定义结构体
    spi_parameter_struct spi_init_struct;
    spi_init_struct.trans_mode        		= SPI_TRANSMODE_FULLDUPLEX;  // 传输模式全双工
    spi_init_struct.device_mode       		= SPI_MASTER;                // 配置为主机
    spi_init_struct.frame_size         		= SPI_FRAMESIZE_8BIT;        // 8位数据
    spi_init_struct.clock_polarity_phase 	= SPI_CK_PL_HIGH_PH_2EDGE;   // 极性相位
    spi_init_struct.nss               		= SPI_NSS_SOFT;              // 软件cs
    spi_init_struct.prescale           		= SPI_PSC_4;                 // SPI时钟预调因数为4
    spi_init_struct.endian            		= SPI_ENDIAN_MSB;            // 高位在前
    //将参数填入SPI0
    spi_init(BSP_SPI, &spi_init_struct);
    //使能SPI
    spi_enable(BSP_SPI);
}

uint8_t spi_read_write_byte(uint8_t dat)
{
    //等待发送缓冲区为空
    while(RESET == spi_i2s_flag_get(BSP_SPI,  SPI_FLAG_TBE) );
    //通过SPI4发送一个字节数据
    spi_i2s_data_transmit(BSP_SPI, dat);

    //等待接收缓冲区不空标志
    while(RESET == spi_i2s_flag_get(BSP_SPI,  SPI_FLAG_RBNE) );
    //读取并返回在SPI4读取到的单字节数据
    return spi_i2s_data_receive(BSP_SPI);
}

//读取芯片ID
//读取设备ID
uint16_t GD25Q32_readID(void)
{
    uint16_t  temp = 0;
    //将CS端拉低为低电平
    W25QXX_CS_ON(0);
    //发送指令90h
    spi_read_write_byte(0x90);//发送读取ID命令
    //发送地址  000000H
    spi_read_write_byte(0x00);
    spi_read_write_byte(0x00);
    spi_read_write_byte(0x00);

    //接收数据
    //接收制造商ID
    temp |= spi_read_write_byte(0xFF)<<8;
    //接收设备ID
    temp |= spi_read_write_byte(0xFF);
    //恢复CS端为高电平
    W25QXX_CS_ON(1);
    //返回ID
    return temp;
}

//发送写使能
void GD25Q32_write_enable(void)
{
    //拉低CS端为低电平
    W25QXX_CS_ON(0);
    //发送指令06h
    spi_read_write_byte(0x06);
    //拉高CS端为高电平
    W25QXX_CS_ON(1);
}

/**********************************************************
 * 函 数 名 称：GD25Q32_wait_busy
 * 函 数 功 能：检测线路是否繁忙
 * 传 入 参 数：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
**********************************************************/
void GD25Q32_wait_busy(void)
{
    unsigned char byte = 0;
    do
    {
        //拉低CS端为低电平
        W25QXX_CS_ON(0);
        //发送指令05h
        spi_read_write_byte(0x05);
        //接收状态寄存器值
        byte = spi_read_write_byte(0Xff);
        //恢复CS端为高电平
        W25QXX_CS_ON(1);
        //判断BUSY位是否为1 如果为1说明在忙，重新读写BUSY位直到为0
    } while( ( byte & 0x01 ) == 1 );
}

/**********************************************************
 * 函 数 名 称：GD25Q32_erase_sector
 * 函 数 功 能：擦除一个扇区
 * 传 入 参 数：addr=擦除的扇区号
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：addr=擦除的扇区号，范围=0~15
**********************************************************/
void GD25Q32_erase_sector(uint32_t addr)
{
    //计算扇区号，一个扇区4KB=4096
    addr *= 4096;
    GD25Q32_write_enable();  //写使能
    GD25Q32_wait_busy();     //判断忙，如果忙则一直等待
    //拉低CS端为低电平
    W25QXX_CS_ON(0);
    //发送指令20h
    spi_read_write_byte(0x20);
    //发送24位扇区地址的高8位
    spi_read_write_byte((uint8_t)((addr)>>16));
    //发送24位扇区地址的中8位
    spi_read_write_byte((uint8_t)((addr)>>8));
    //发送24位扇区地址的低8位
    spi_read_write_byte((uint8_t)addr);
    //恢复CS端为高电平
    W25QXX_CS_ON(1);
    //等待擦除完成
    GD25Q32_wait_busy();
}

/**********************************************************
 * 函 数 名 称：GD25Q32_write
 * 函 数 功 能：写数据到GD25Q32进行保存
 * 传 入 参 数：buffer=写入的数据内容        addr=写入地址        numbyte=写入数据的长度
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
**********************************************************/
void GD25Q32_write(uint8_t* buffer, uint32_t addr, uint16_t numbyte)
{
    unsigned int i = 0;
    //擦除扇区数据
    GD25Q32_erase_sector(addr/4096);
    //写使能
    GD25Q32_write_enable();
    //忙检测
    GD25Q32_wait_busy();
    //写入数据
    //拉低CS端为低电平
    W25QXX_CS_ON(0);
    //发送指令02h
    spi_read_write_byte(0x02);
    //发送写入的24位地址中的高8位
    spi_read_write_byte((uint8_t)((addr)>>16));
    //发送写入的24位地址中的中8位
    spi_read_write_byte((uint8_t)((addr)>>8));
    //发送写入的24位地址中的低8位
    spi_read_write_byte((uint8_t)addr);
    //根据写入的字节长度连续写入数据buffer
    for(i=0; i<numbyte; i++)
    {
        spi_read_write_byte(buffer[i]);
    }
    //恢复CS端为高电平
    W25QXX_CS_ON(1);
    //忙检测
    GD25Q32_wait_busy();
}

/**********************************************************
 * 函 数 名 称：GD25Q32_read
 * 函 数 功 能：读取GD25Q32的数据
 * 传 入 参 数：buffer=读出数据的保存地址  read_addr=读取地址   read_length=读去长度
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
**********************************************************/
void GD25Q32_read(uint8_t* buffer,uint32_t read_addr,uint16_t read_length)
{
    uint16_t i;
    //拉低CS端为低电平
    W25QXX_CS_ON(0);
    //发送指令03h
    spi_read_write_byte(0x03);
    //发送24位读取数据地址的高8位
    spi_read_write_byte((uint8_t)((read_addr)>>16));
    //发送24位读取数据地址的中8位
    spi_read_write_byte((uint8_t)((read_addr)>>8));
    //发送24位读取数据地址的低8位
    spi_read_write_byte((uint8_t)read_addr);
    //根据读取长度读取出地址保存到buffer中
    for(i=0; i<read_length; i++)
    {
        buffer[i]= spi_read_write_byte(0XFF);
    }
    //恢复CS端为高电平
    W25QXX_CS_ON(1);
}
