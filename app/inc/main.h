#ifndef __MAIN_H_
#define __MAIN_H_


#include "gd32f4xx.h"
#include "stdint.h"
#include "stdio.h"
#include "gd32f4xx_libopt.h"
#include "gd32f4xx_exti.h"




//#include "soft_timer.h"
//#include "buzzer.h"

void board_init(void);
uint32_t get_system_tick(void);
void delay_us(uint32_t _us);
void delay_ms(uint32_t _ms);

#endif


 

