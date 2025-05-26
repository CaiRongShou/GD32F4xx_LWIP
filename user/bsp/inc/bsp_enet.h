#ifndef GD32F4xx_ENET_EVAL_H
#define GD32F4xx_ENET_EVAL_H

#include "main.h"


//#define USE_DHCP 1 /* enable DHCP, if disabled static address is used */

#define USE_ENET_INTERRUPT
//#define TIMEOUT_CHECK_USE_LWIP
/* MAC address: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
#define MAC_ADDR0 2
#define MAC_ADDR1 0xA
#define MAC_ADDR2 0xF
#define MAC_ADDR3 0xE
#define MAC_ADDR4 0xD
#define MAC_ADDR5 6


/* MII and RMII mode selection */
#define RMII_MODE // user have to provide the 50 MHz clock by soldering a 50 MHz oscillator


/*lwip控制结构体*/
typedef struct 
{
uint8_t mac[6]; /* MAC地址 */
uint8_t remoteip[4]; /* 远端主机IP地址 */ 
uint8_t ip[4]; /* 本机IP地址 */
uint8_t netmask[4]; /* 子网掩码 */
uint8_t gateway[4]; /* 默认网关的IP地址 */
}__lwip_dev;

extern __lwip_dev lwipdev; /* lwip控制结构体 */

/* function declarations */
#ifdef USE_DHCP
void lwip_dhcp_process_handle(void);
#endif /* USE_DHCP */

/* setup ethernet system(GPIOs, clocks, MAC, DMA, systick) */
void enet_system_setup(void);

void lwip_stack_init(void);

void lwip_periodic_handle(__IO uint32_t localtime);

void lwip_pkt_handle(void);

#endif /* GD32F4xx_ENET_EVAL_H */
