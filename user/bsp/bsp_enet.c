#include "gd32f4xx_enet.h"
#include "bsp_enet.h"
#include "main.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/init.h"
#include "netif/etharp.h"
#include "lwip/dhcp.h"
#include "ethernetif.h"
#include "stdint.h"
#include <stdio.h>
#include "lwip/priv/tcp_priv.h"
#include "lwip/timeouts.h"

#define MAX_DHCP_TRIES        4


typedef enum {
    DHCP_START = 0,
    DHCP_WAIT_ADDRESS,
    DHCP_ADDRESS_ASSIGNED,
    DHCP_TIMEOUT
} dhcp_state_enum;

#ifdef USE_DHCP
uint32_t dhcp_fine_timer = 0;
uint32_t dhcp_coarse_timer = 0;
dhcp_state_enum dhcp_state = DHCP_START;
#endif /* USE_DHCP */

struct netif g_mynetif;
uint32_t tcp_timer = 0;
uint32_t arp_timer = 0;
ip_addr_t ip_address = {0};

__lwip_dev lwipdev;                 /* lwip控制结构体 */

static __IO uint32_t enet_init_status = 0;
static void enet_gpio_config(void);
static void enet_mac_dma_config(void);
#ifdef USE_ENET_INTERRUPT
static void nvic_configuration(void);
#endif /* USE_ENET_INTERRUPT */

/*!
    \brief      setup ethernet system(GPIOs, clocks, MAC, DMA, systick)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void enet_system_setup(void)
{
#ifdef USE_ENET_INTERRUPT
    nvic_configuration();
#endif /* USE_ENET_INTERRUPT */

    /* configure the GPIO ports for ethernet pins */
    enet_gpio_config();

    /* configure the ethernet MAC/DMA */
    enet_mac_dma_config();

    if(0 == enet_init_status) {
        while(1) {
        }
    }

#ifdef USE_ENET_INTERRUPT
    enet_interrupt_enable(ENET_DMA_INT_NIE);
    enet_interrupt_enable(ENET_DMA_INT_RIE);
#endif /* USE_ENET_INTERRUPT */

#ifdef SELECT_DESCRIPTORS_ENHANCED_MODE
    enet_desc_select_enhanced_mode();
#endif /* SELECT_DESCRIPTORS_ENHANCED_MODE */
}

/*!
    \brief      configures the ethernet interface
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void enet_mac_dma_config(void)
{
    ErrStatus reval_state = ERROR;

    /* enable ethernet clock  */
    rcu_periph_clock_enable(RCU_ENET);
    rcu_periph_clock_enable(RCU_ENETTX);
    rcu_periph_clock_enable(RCU_ENETRX);

    /* reset ethernet on AHB bus */
    enet_deinit();

    reval_state = enet_software_reset();
    if(ERROR == reval_state) {
        while(1) {}
    }

    /* configure the parameters which are usually less cared for enet initialization */
//  enet_initpara_config(HALFDUPLEX_OPTION, ENET_CARRIERSENSE_ENABLE|ENET_RECEIVEOWN_ENABLE|ENET_RETRYTRANSMISSION_DISABLE|ENET_BACKOFFLIMIT_10|ENET_DEFERRALCHECK_DISABLE);
//  enet_initpara_config(DMA_OPTION, ENET_FLUSH_RXFRAME_ENABLE|ENET_SECONDFRAME_OPT_ENABLE|ENET_NORMAL_DESCRIPTOR);

#ifdef CHECKSUM_BY_HARDWARE
    enet_init_status = enet_init(ENET_AUTO_NEGOTIATION, ENET_AUTOCHECKSUM_DROP_FAILFRAMES, ENET_BROADCAST_FRAMES_PASS);
#else
    enet_init_status = enet_init(ENET_AUTO_NEGOTIATION, ENET_NO_AUTOCHECKSUM, ENET_BROADCAST_FRAMES_PASS);
#endif /* CHECKSUM_BY_HARDWARE */

}

#ifdef USE_ENET_INTERRUPT
/*!
    \brief      configures the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void nvic_configuration(void)
{
    nvic_irq_enable(ENET_IRQn, 2U, 2U);
}
#endif /* USE_ENET_INTERRUPT */

/*!
    \brief      configures the different GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void enet_gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);

    /* enable SYSCFG clock */
    rcu_periph_clock_enable(RCU_SYSCFG);

    /* choose DIV2 to get 50MHz from 200MHz on CKOUT0 pin (PA8) to clock the PHY */
    rcu_ckout0_config(RCU_CKOUT0SRC_PLLP, RCU_CKOUT0_DIV4);
    syscfg_enet_phy_interface_config(SYSCFG_ENET_PHY_RMII);


    /* PA1: ETH_RMII_REF_CLK */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_1);

    /* PA2: ETH_MDIO */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_2);

    /* PA7: ETH_RMII_CRS_DV */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_7);

    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_1);
    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_2);
    gpio_af_set(GPIOA, GPIO_AF_11, GPIO_PIN_7);

    /* PB11: ETH_RMII_TX_EN */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_11);

    /* PB13: ETH_RMII_TXD1 */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_13);

    /* PB12: ETH_RMII_TXD0 */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_12);

    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_11);
    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_13);
    gpio_af_set(GPIOB, GPIO_AF_11, GPIO_PIN_12);

    /* PC1: ETH_MDC */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_1);

    /* PC4: ETH_RMII_RXD0 */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_4);

    /* PC5: ETH_RMII_RXD1 */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_5);

    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_1);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_4);
    gpio_af_set(GPIOC, GPIO_AF_11, GPIO_PIN_5);
}



uint32_t tempreg;
uint8_t set_lwip_10m;
uint8_t lan_is_disconnect;


/**
 * @brief   read_lan_phy
 *          读取PHY芯片对应寄存器值
 * @param   无
 * @retval  无
 * @note    
 * @since   
*/
unsigned short read_lan_phy(unsigned short phy_addr, unsigned short phy_Reg)
{
	uint16_t val;
	enet_phy_write_read(ENET_PHY_READ,phy_addr, phy_Reg,&val);
	
	return val;	
}

/**
 * @brief   write_lan_phy
 *          写入PHY芯片对应寄存器值
 * @param   无
 * @retval  无
 * @note    
 * @since   
*/
void write_lan_phy(unsigned short phy_addr, unsigned short phy_Reg, unsigned short phy_value)
{
	 enet_phy_write_read(ENET_PHY_WRITE,phy_addr,phy_Reg , &phy_value);
}

/*! @brief  PHY 芯片初始化 */
void lan_phy_init(void)
{
    volatile int i,ii;
    __IO uint32_t timeout = 0;
    uint32_t media_temp = 0U,reg_value=0;
    volatile int delaytimes = 5 ;
    lan_is_disconnect=0;
    uint16_t RegVlaue=0;

	if(read_lan_phy(PHY_ADDRESS,2)==0x0243)
	{
		if(set_lwip_10m)
		{
			write_lan_phy(PHY_ADDRESS, 0, 0x8100);
			delay_ms(20); 								//ljh20210625	
			write_lan_phy(PHY_ADDRESS, 0, 0x0100);//ljh20210625	
            delay_ms(100);            
			for(ii=0;ii<20;ii++)
			{ 
			  for(i=0;i<5;i++)
				{
                    delay_ms(20);
                    if(read_lan_phy(PHY_ADDRESS,1 )& 0x0020) break;
                }						  	
			} 			
			RegVlaue = read_lan_phy(PHY_ADDRESS, 1);
			if((RegVlaue & 1<<2)==0)//没有接网线
			{	  
				lan_is_disconnect=1;
			}		
//			G101_10M_jmp:
			write_lan_phy(PHY_ADDRESS, 4, 0x0041);
			RegVlaue=read_lan_phy(PHY_ADDRESS,4);			
			tempreg = 0x0014;
			goto set_jump;
		}
		else
		{
			write_lan_phy(PHY_ADDRESS, 0, 0x9200);
			delay_ms(100);
			for(ii=0;ii<100;ii++)
			{ 
			  for(i=0;i<100;i++)
			  {
                  delay_ms(1);  
				  RegVlaue = read_lan_phy(PHY_ADDRESS, 1 );
			  	if(read_lan_phy(PHY_ADDRESS,1)& 0x0004) break;
			  }                         
			} 
			
			for(ii=0;ii<100;ii++)
			{ 
			  for(i=0;i<100;i++)
			  {
                  delay_ms(1);  
				  RegVlaue = read_lan_phy(PHY_ADDRESS, 1 );
			  	if(read_lan_phy(PHY_ADDRESS,1)& 0x0020) break;
			  }                         
			} 
			
//			RegVlaue = read_lan_phy(PHY_ADDRESS, 1 );
//			if((RegVlaue & 0x0004) == 0)//没有接网线
//			{	  
//	    		lan_is_disconnect=1;
//			}
//			RegVlaue = read_lan_phy(PHY_ADDRESS,1);																
//			if((RegVlaue & 0x0020) == 0)
//			{
//				write_lan_phy(PHY_ADDRESS, 0, 0x8100);				
//				delay_ms(20);  
//				goto G101_10M_jmp;
//			}

			tempreg=0x00;
			if((read_lan_phy(PHY_ADDRESS,0)& (1<<13)))
				tempreg+=0x0008;  //100M
			else
				tempreg+=0x0004;  //10M
	
			if((read_lan_phy(PHY_ADDRESS,0)& (1<<8)))			
				tempreg+=0x0010;   //full		
		}
	}
	else{
        tempreg=0x0018;
		media_temp |= ENET_SPEEDMODE_100M;
		media_temp |= ENET_MODE_FULLDUPLEX;	        
	}
set_jump: 
		if(tempreg == 0x0018)//100fdx
		{
			//Lan_is_100M=1;
				media_temp |= ENET_SPEEDMODE_100M;
				media_temp |= ENET_MODE_FULLDUPLEX;	
		}
		else if(tempreg == 0x0008) { //100hdx  ETH_MAC_Mode
		//Lan_is_100M=1;
				media_temp |= ENET_SPEEDMODE_100M;
				media_temp |= ENET_MODE_HALFDUPLEX;		
		} 
		else if(tempreg == 0x0014) {  //10fdx
				media_temp |= ENET_SPEEDMODE_10M;
				media_temp |= ENET_MODE_FULLDUPLEX;	
		} 
		else if(tempreg == 0x0004) { //10hdx
			set_10hdx:
				media_temp |= ENET_SPEEDMODE_10M;
				media_temp |= ENET_MODE_HALFDUPLEX;	      
		}
		else{
			goto set_10hdx;
		}
		
	  reg_value = ENET_MAC_CFG;
    /* configure ENET_MAC_CFG register */
    reg_value &= (~(ENET_MAC_CFG_SPD |ENET_MAC_CFG_DPM |ENET_MAC_CFG_LBM));
    reg_value |= media_temp;
    ENET_MAC_CFG = reg_value;		
   // Lan_init_is_ok=1;	  
}






/**
 * @breif       lwip 默认IP设置
 * @param       lwipx  : lwip控制结构体指针
 * @retval      无
 */
static void lwip_comm_default_ip_set(__lwip_dev *lwipx)
{
    /* 默认远端IP为:192.168.1.134 */
    lwipx->remoteip[0] = 192;
    lwipx->remoteip[1] = 168;
    lwipx->remoteip[2] = 1;
    lwipx->remoteip[3] = 27;
    
    /* MAC地址设置(高三字节固定为:2.0.0) */
    lwipx->mac[0] = 2;       
    lwipx->mac[1] = 0xA;
    lwipx->mac[2] = 0xF;
    lwipx->mac[3] = 0xE;      
    lwipx->mac[4] = 0xD;
    lwipx->mac[5] = 6;
    
    /* 默认本地IP为:192.168.1.30 */
    lwipx->ip[0] = 192;
    lwipx->ip[1] = 168;
    lwipx->ip[2] = 1;
    lwipx->ip[3] = 123;
    /* 默认子网掩码:255.255.255.0 */
    lwipx->netmask[0] = 255;
    lwipx->netmask[1] = 255;
    lwipx->netmask[2] = 255;
    lwipx->netmask[3] = 0;

   /* 默认网关:192.168.1.1 */
    lwipx->gateway[0] = 192;
    lwipx->gateway[1] = 168;
    lwipx->gateway[2] = 1;
    lwipx->gateway[3] = 1;
}



/*!
    \brief      initializes the LwIP stack
    \param[in]  none
    \param[out] none
    \retval     none
*/
void lwip_stack_init(void)
{
    ip_addr_t ipaddr;
    ip_addr_t netmask;
    ip_addr_t gw;

    /* initializes the dynamic memory heap defined by MEM_SIZE */
    mem_init();

    /* initializes the memory pools defined by MEMP_NUM_x */
    memp_init();
	
//	lwip_init();                                /* 初始化LWIP内核 */
#ifdef TIMEOUT_CHECK_USE_LWIP
    sys_timeouts_init();
#endif /* TIMEOUT_CHECK_USE_LWIP */

#ifdef USE_DHCP
    ipaddr.addr = 0;
    netmask.addr = 0;
    gw.addr = 0;
#else
    lwip_comm_default_ip_set(&lwipdev);
	
	IP4_ADDR(&(g_mynetif.ip_addr), lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3]);
	IP4_ADDR(&(g_mynetif.netmask), lwipdev.netmask[0], lwipdev.netmask[1], lwipdev.netmask[2], lwipdev.netmask[3]);
	IP4_ADDR(&(g_mynetif.gw), lwipdev.gateway[0], lwipdev.gateway[1], lwipdev.gateway[2], lwipdev.gateway[3]);

	printf("网卡en的MAC地址为:................%d.%d.%d.%d.%d.%d\r\n", lwipdev.mac[0], lwipdev.mac[1], lwipdev.mac[2], lwipdev.mac[3], lwipdev.mac[4], lwipdev.mac[5]);

	printf("静态IP地址........................%d.%d.%d.%d\r\n", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3]);

	printf("子网掩码..........................%d.%d.%d.%d\r\n", lwipdev.netmask[0], lwipdev.netmask[1], lwipdev.netmask[2], lwipdev.netmask[3]);

	printf("默认网关..........................%d.%d.%d.%d\r\n", lwipdev.gateway[0], lwipdev.gateway[1], lwipdev.gateway[2], lwipdev.gateway[3]);
	
#endif /* USE_DHCP */

    /* - netif_add(struct netif *netif, ip_addr_t *ipaddr,
              ip_addr_t *netmask, ip_addr_t *gw,
              void *state, err_t (* init)(struct netif *netif),
              err_t (* input)(struct pbuf *p, struct netif *netif))

     Adds your network interface to the netif_list. Allocate a struct
    netif and pass a pointer to this structure as the first argument.
    Give pointers to cleared ip_addr structures when using DHCP,
    or fill them with sane numbers otherwise. The state pointer may be NULL.

    The init function pointer must point to a initialization function for
    your ethernet netif interface. The following code illustrates it's use.*/

    netif_add(&g_mynetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);
    /* registers the default network interface */
    netif_set_default(&g_mynetif);
#ifdef USE_DHCP 
    dhcp_start(&g_mynetif); 
#else
    /* when the netif is fully configured this function must be called */
    netif_set_up(&g_mynetif);
#endif /* USE_DHCP */
}

/*!
    \brief      called when a frame is received
    \param[in]  none
    \param[out] none
    \retval     none
*/
void lwip_pkt_handle(void)
{
    /* read a received packet from the Ethernet buffers and send it to the lwIP for handling */
    ethernetif_input(&g_mynetif);
}

/*!
    \brief      LwIP periodic tasks
    \param[in]  localtime the current LocalTime value
    \param[out] none
    \retval     none
*/
void lwip_periodic_handle(__IO uint32_t localtime)
{
#if LWIP_TCP
    /* TCP periodic process every 250 ms */
    if(localtime - tcp_timer >= TCP_TMR_INTERVAL) {
        tcp_timer =  localtime;
        tcp_tmr();
    }

#endif /* LWIP_TCP */

    /* ARP periodic process every 5s */
    if((localtime - arp_timer) >= ARP_TMR_INTERVAL) {
        arp_timer = localtime;
        etharp_tmr();
    }

#ifdef USE_DHCP
    /* fine DHCP periodic process every 500ms */
    if(localtime - dhcp_fine_timer >= DHCP_FINE_TIMER_MSECS) {
        dhcp_fine_timer =  localtime;
        dhcp_fine_tmr();
        if((DHCP_ADDRESS_ASSIGNED != dhcp_state) && (DHCP_TIMEOUT != dhcp_state)) {
            /* process DHCP state machine */
            lwip_dhcp_process_handle();
        }
    }

    /* DHCP coarse periodic process every 60s */
    if(localtime - dhcp_coarse_timer >= DHCP_COARSE_TIMER_MSECS) {
        dhcp_coarse_timer =  localtime;
        dhcp_coarse_tmr();
    }

#endif /* USE_DHCP */
}

#ifdef USE_DHCP
/*!
    \brief      lwip_dhcp_process_handle
    \param[in]  none
    \param[out] none
    \retval     none
*/
void lwip_dhcp_process_handle(void)
{
    ip_addr_t ipaddr;
    ip_addr_t netmask;
    ip_addr_t gw;
    struct dhcp *dhcp_client;

    switch(dhcp_state) {
    case DHCP_START:
        dhcp_start(&g_mynetif);

        dhcp_state = DHCP_WAIT_ADDRESS;
        break;

    case DHCP_WAIT_ADDRESS:
        /* read the new IP address */
        ip_address.addr = g_mynetif.ip_addr.addr;

        if(0 != ip_address.addr) {
            dhcp_state = DHCP_ADDRESS_ASSIGNED;

            printf("\r\nDHCP -- eval board ip address: %d.%d.%d.%d \r\n", ip4_addr1_16(&ip_address), \
                   ip4_addr2_16(&ip_address), ip4_addr3_16(&ip_address), ip4_addr4_16(&ip_address));
        } else {
            /* DHCP timeout */
            if(dhcp_client->tries > MAX_DHCP_TRIES) {
                dhcp_state = DHCP_TIMEOUT;
                /* stop DHCP */
                dhcp_stop(&g_mynetif);

                /* static address used */
                IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
                IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
                IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
                netif_set_addr(&g_mynetif, &ipaddr, &netmask, &gw);
				printf("DHCP服务超时,使用静态IP地址!\r\n");

                printf("网卡en的MAC地址为:................%d.%d.%d.%d.%d.%d\r\n", lwipdev.mac[0], lwipdev.mac[1], lwipdev.mac[2], lwipdev.mac[3], lwipdev.mac[4], lwipdev.mac[5]);

                printf("静态IP地址........................%d.%d.%d.%d\r\n", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3]);

                printf("子网掩码..........................%d.%d.%d.%d\r\n", lwipdev.netmask[0], lwipdev.netmask[1], lwipdev.netmask[2], lwipdev.netmask[3]);

                printf("默认网关..........................%d.%d.%d.%d\r\n", lwipdev.gateway[0], lwipdev.gateway[1], lwipdev.gateway[2], lwipdev.gateway[3]);
            }
        }
        break;

    default:
        break;
    }
}
#endif /* USE_DHCP */

unsigned long sys_now(void)
{
    return get_system_tick();
}