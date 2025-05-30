#ifndef __SOFT_TIMER_H__
#define __SOFT_TIMER_H__

#include "main.h"

// 定义软件定时器的枚举类型
typedef enum
{
    SOFT_TIMER_0,
    SOFT_TIMER_1,
    SOFT_TIMER_2,
    SOFT_TIMER_3,
    SOFT_TIMER_4,
    SOFT_TIMER_5,
    SOFT_TIMER_6,
    SOFT_TIMER_7,
    SOFT_TIMER_8,
    SOFT_TIMER_9,
    SOFT_TIMER_MAX // 定时器的最大数量
} soft_timer_type;

// 定义软件定时器的结构体
typedef struct
{
    volatile uint32_t counter;   // 计数器
    volatile uint32_t timeout;   // 超时时间
    volatile uint8_t is_timeout; // 超时标志
    volatile uint8_t is_repeat;  // 是否重复计数标志(为1是重复定时器)
} software_timer;


void soft_timer_single_init(soft_timer_type timer, uint32_t timeout);
void soft_timer_repeat_init(soft_timer_type timer, uint32_t timeout);
void soft_timer_start(soft_timer_type timer);
void soft_timer_stop(soft_timer_type timer);
uint8_t soft_timer_is_timeout(soft_timer_type timer);
void soft_timer_reset(soft_timer_type timer);
void soft_timer_tick(void);
static void soft_timer_interrupt_disable(void);
static void soft_timer_interrupt_enable(void);

#endif // __SOFT_TIMER_H__
