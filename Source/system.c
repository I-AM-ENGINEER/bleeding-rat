#include "stm32f4xx.h"
#include "system.h"
#include "shell.h"
#include "move.h"

extern TIM_HandleTypeDef SYSTEM_TIMER_US;

inline uint64_t time_us( void ){
    return ((uint64_t)time_ms() * 1000) + (uint64_t)__HAL_TIM_GET_COUNTER(&SYSTEM_TIMER_US);
}

inline uint32_t time_ms( void ){
    return HAL_GetTick();
}

int32_t time_us_init( void ){
    HAL_TIM_Base_Start(&SYSTEM_TIMER_US);
    return 0;
}

void delay_us( uint32_t us ){
    uint32_t start_us = (uint32_t)time_us();
    while(((uint32_t)time_us() - start_us) < us){};
}

void delay_ms( uint32_t ms ){
    uint32_t start_ms = (uint32_t)time_ms();
    while((time_ms() - start_ms) < ms){
        shell_process();
    };
}

int32_t sys_init( void ){
    time_us_init();
	shell_init();
    shell_log("[timus] us timer init ok");
    shell_log("[shell] init ok");
	move_init();
	shell_log("[move] init ok");
}
