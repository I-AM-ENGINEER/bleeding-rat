#include "stm32f4xx.h"
#include "cmsis_os.h"
#include "system.h"
#include "shell.h"
#include "core.h"
#include "move.h"

typedef StaticTask_t osStaticThreadDef_t;

osThreadId_t shellTaskHandle;
uint32_t shellTaskBuffer[ 128 ];
osStaticThreadDef_t shellTaskControlBlock;
const osThreadAttr_t shellTask_attributes = {
  .name = "shellTask",
  .cb_mem = &shellTaskControlBlock,
  .cb_size = sizeof(shellTaskControlBlock),
  .stack_mem = &shellTaskBuffer[0],
  .stack_size = sizeof(shellTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};

void shellTask( void *args );

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

inline void delay( uint32_t ms ){
    osDelay(ms);
}

void sys_init( void ){
	shell_init();
    shell_log("[shell] init ok");
    osThreadNew(shellTask, NULL, &shellTask_attributes);
    time_us_init();
    shell_log("[timus] us timer init ok");
    core_init();
    while(1){
        core_loop();
    }
}

void shellTask( void *args ){
    shell_log("[shell] task started");
    while(1){
        shell_process();
        osDelay(10);
    }
}
