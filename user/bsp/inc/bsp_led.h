#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include "main.h"

typedef enum
{
    LED1 = 0,
    LED_MAX_NUM
} led_type_def;

#define LED1_PIN                         GPIO_PIN_2
#define LED1_GPIO_PORT                   GPIOB
#define LED1_GPIO_CLK                    RCU_GPIOB


void bsp_led_init(void);
void bsp_led_on(led_type_def led);
void bsp_led_off(led_type_def led);
void bsp_led_toggle(led_type_def led);
uint8_t bsp_led_get_status(led_type_def led);

#endif /* __BSP_LED_H__ */
