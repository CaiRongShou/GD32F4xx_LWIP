#include "main.h"
#include "bsp_uart.h"
#include "bsp_enet.h"



int main(void)
{
	board_init();
	
	bsp_uart_init();
	
	enet_system_setup();
	
	lwip_stack_init();
	
	printf("init susccess\r\n");
    while(1) 
	{
		#ifndef USE_ENET_INTERRUPT
		if(enet_rxframe_size_get())
			lwip_pkt_handle();
		#endif
        lwip_periodic_handle(get_system_tick());
    }
}

