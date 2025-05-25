#include "bsp_sd.h"
#include <stdio.h>


sd_card_info_struct sd_cardinfo;                            /* information of SD card */
uint32_t buf_write[512];                                    /* store the data written to the card */
uint32_t buf_read[512];                                     /* store the data read from the card */



/*!
    \brief      configure the NVIC
    \param[in]  none
    \param[out] none
    \retval     none
*/
void sd_nvic_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);
    nvic_irq_enable(SDIO_IRQn, 5, 0);
}

/*!
    \brief      initialize the card, get the card information, set the bus mode and transfer mode
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_io_init(void)
{
    sd_error_enum status = SD_OK;
    uint32_t cardstate = 0;

    sd_nvic_config();

    status = sd_init();
    if(SD_OK == status) {
        status = sd_card_information_get(&sd_cardinfo);
    }
    if(SD_OK == status) {
        status = sd_card_select_deselect(sd_cardinfo.card_rca);
    }
    status = sd_cardstatus_get(&cardstate);
    if(cardstate & 0x02000000) {
        printf("\r\n the card is locked!");
        while(1) {
        }
    }
    if((SD_OK == status) && (!(cardstate & 0x02000000))) {
        /* set bus mode */
        status = sd_bus_mode_config(SDIO_BUSMODE_4BIT);
//        status = sd_bus_mode_config(SDIO_BUSMODE_1BIT);
    }
    if(SD_OK == status) {
        /* set data transfer mode */
        status = sd_transfer_mode_config(SD_DMA_MODE);
//        status = sd_transfer_mode_config(SD_POLLING_MODE);
    }
    return status;
}

/*!
    \brief      get the card information and print it out by USRAT
    \param[in]  none
    \param[out] none
    \retval     none
*/
void card_info_get(void)
{
    uint8_t sd_spec, sd_spec3, sd_spec4, sd_security;
    uint32_t block_count, block_size;
    uint16_t temp_ccc;

    printf("\r\n Card information:");
    sd_spec = (sd_scr[1] & 0x0F000000) >> 24;
    sd_spec3 = (sd_scr[1] & 0x00008000) >> 15;
    sd_spec4 = (sd_scr[1] & 0x00000400) >> 10;
    if(2 == sd_spec) {
        if(1 == sd_spec3) {
            if(1 == sd_spec4) {
                printf("\r\n## Card version 4.xx ##");
            } else {
                printf("\r\n## Card version 3.0x ##");
            }
        } else {
            printf("\r\n## Card version 2.00 ##");
        }
    } else if(1 == sd_spec) {
        printf("\r\n## Card version 1.10 ##");
    } else if(0 == sd_spec) {
        printf("\r\n## Card version 1.0x ##");
    }

    sd_security = (sd_scr[1] & 0x00700000) >> 20;
    if(2 == sd_security) {
        printf("\r\n## SDSC card ##");
    } else if(3 == sd_security) {
        printf("\r\n## SDHC card ##");
    } else if(4 == sd_security) {
        printf("\r\n## SDXC card ##");
    }

    block_count = (sd_cardinfo.card_csd.c_size + 1) * 1024;
    block_size = 512;
    printf("\r\n## Device size is %dKB ##", sd_card_capacity_get());
    printf("\r\n## Block size is %dB ##", block_size);
    printf("\r\n## Block count is %d ##", block_count);

    if(sd_cardinfo.card_csd.read_bl_partial) {
        printf("\r\n## Partial blocks for read allowed ##");
    }
    if(sd_cardinfo.card_csd.write_bl_partial) {
        printf("\r\n## Partial blocks for write allowed ##");
    }
    temp_ccc = sd_cardinfo.card_csd.ccc;
    printf("\r\n## CardCommandClasses is: %x ##", temp_ccc);
    if((SD_CCC_BLOCK_READ & temp_ccc) && (SD_CCC_BLOCK_WRITE & temp_ccc)) {
        printf("\r\n## Block operation supported ##");
    }
    if(SD_CCC_ERASE & temp_ccc) {
        printf("\r\n## Erase supported ##");
    }
    if(SD_CCC_WRITE_PROTECTION & temp_ccc) {
        printf("\r\n## Write protection supported ##");
    }
    if(SD_CCC_LOCK_CARD & temp_ccc) {
        printf("\r\n## Lock unlock supported ##");
    }
    if(SD_CCC_APPLICATION_SPECIFIC & temp_ccc) {
        printf("\r\n## Application specific supported ##");
    }
    if(SD_CCC_IO_MODE & temp_ccc) {
        printf("\r\n## I/O mode supported ##");
    }
    if(SD_CCC_SWITCH & temp_ccc) {
        printf("\r\n## Switch function supported ##");
    }
}

/*
//	函数：FatFs_FileTest
//	功能：进行文件写入和读取测试
//
uint8_t  FatFs_FileTest(void)	//文件创建和写入测试
{
    uint8_t i = 0;
    uint16_t BufferSize = 0;
    FIL	MyFile;			// 文件对象
    UINT 	MyFile_Num;		//	数据长度
    BYTE 	MyFile_WriteBuffer[] = "STM32H7B0 SD卡 文件系统测试";	//要写入的数据
    BYTE 	MyFile_ReadBuffer[1024];	//要读出的数据
    uint8_t MyFile_Res;   // Return value for SD

    printf("-------------FatFs 文件创建和写入测试---------------\r\n");

    //挂载文件系统
    FATFS fs;
    MyFile_Res = f_mount(&fs, "", 1);
    if (MyFile_Res != FR_OK) {
        return MyFile_Res;
    }

    MyFile_Res = f_open(&MyFile,"0:Test.txt",FA_CREATE_ALWAYS | FA_WRITE);	//打开文件，若不存在则创建该文件
    if(MyFile_Res == FR_OK)
    {
        printf("文件打开/创建成功，准备写入数据...\r\n");

        MyFile_Res = f_write(&MyFile,MyFile_WriteBuffer,sizeof(MyFile_WriteBuffer),&MyFile_Num);	//向文件写入数据
        if (MyFile_Res == FR_OK)
        {
            printf("写入成功，写入内容为：\r\n");
            printf("%s\r\n",MyFile_WriteBuffer);
        }
        else
        {
            printf("文件写入失败，请检查SD卡或重新格式化!\r\n");
            f_close(&MyFile);	  //关闭文件
            return ERROR;
        }
        f_close(&MyFile);	  //关闭文件
    }
    else
    {
        printf("无法打开/创建文件，请检查SD卡或重新格式化!\r\n");
        f_close(&MyFile);	  //关闭文件
        return ERROR;
    }

    printf("-------------FatFs 文件读取测试---------------\r\n");

    BufferSize = sizeof(MyFile_WriteBuffer)/sizeof(BYTE);									// 计算写入的数据长度
    MyFile_Res = f_open(&MyFile,"0:Test.txt",FA_OPEN_EXISTING | FA_READ);	//打开文件，若不存在则创建该文件
    MyFile_Res = f_read(&MyFile,MyFile_ReadBuffer,BufferSize,&MyFile_Num);			// 读取文件
    if(MyFile_Res == FR_OK)
    {
        printf("文件读取成功，正在校验数据...\r\n");

        for(i=0; i<BufferSize; i++)
        {
            if(MyFile_WriteBuffer[i] != MyFile_ReadBuffer[i])		// 校验数据
            {
                printf("校验失败，请检查SD卡或重新格式化!\r\n");
                f_close(&MyFile);	  //关闭文件
                return ERROR;
            }
        }
        printf("校验成功，读出的数据为：\r\n");
        printf("%s\r\n",MyFile_ReadBuffer);
    }
    else
    {
        printf("无法读取文件，请检查SD卡或重新格式化!\r\n");
        f_close(&MyFile);	  //关闭文件
        return ERROR;
    }

    f_close(&MyFile);	  //关闭文件
    f_mount(NULL, "", 0);
    return SUCCESS;
}
*/

