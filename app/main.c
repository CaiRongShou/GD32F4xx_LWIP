#include "main.h"  
#include "bsp_enet.h"
#include "netconf.h"



int main(void)
{
	board_init();
	
	/* setup ethernet system(GPIOs, clocks, MAC, DMA, systick) */
    enet_system_setup(); 

    /* initilaize the LwIP stack */
    lwip_stack_init();
	printf("init susccess\r\n");
    while(1) 
	{

#ifndef USE_ENET_INTERRUPT
        /* check if any packet received */
        if(enet_rxframe_size_get()) {
            /* process received ethernet packet */
            lwip_pkt_handle();
        }
#endif /* USE_ENET_INTERRUPT */

        /* handle periodic timers for LwIP */
#ifdef TIMEOUT_CHECK_USE_LWIP
        sys_check_timeouts();

#ifdef USE_DHCP
        lwip_dhcp_process_handle();
#endif /* USE_DHCP */

#else
        lwip_periodic_handle(get_system_tick());
#endif /* TIMEOUT_CHECK_USE_LWIP */
    }
}

