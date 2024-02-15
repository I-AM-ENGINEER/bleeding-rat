#include "stm32f4xx.h"
#include "main.h"
#include "cmsis_os.h"
#include "system.h"
#include "shell.h"
#include "core.h"
#include "move.h"
#include "imu.h"
#include "collision.h"

typedef StaticTask_t osStaticThreadDef_t;

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

osThreadId_t collisionTaskHandle;
uint32_t collisionTaskBuffer[ 128 ];
osStaticThreadDef_t collisionTaskControlBlock;
const osThreadAttr_t collisionTask_attributes = {
  .name = "collisionTask",
  .cb_mem = &collisionTaskControlBlock,
  .cb_size = sizeof(collisionTaskControlBlock),
  .stack_mem = &collisionTaskBuffer[0],
  .stack_size = sizeof(collisionTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh4,
};

void shellTask( void *args );
void collisionTask( void *args );

static bool encoder_initialised = false;

extern TIM_HandleTypeDef SYSTEM_TIM_US;
extern TIM_HandleTypeDef COLLISION_TIM_ADC_TRIGGER;
extern ADC_HandleTypeDef COLLISION_ADC;

inline uint64_t time_us( void ){
    return ((uint64_t)time_ms() * 1000) + (uint64_t)__HAL_TIM_GET_COUNTER(&SYSTEM_TIM_US);
}

inline uint32_t time_ms( void ){
    return HAL_GetTick();
}

int32_t time_us_init( void ){
    HAL_TIM_Base_Start(&SYSTEM_TIM_US);
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
    int res = 0;
    res = shell_init();
    if(res == 0){
        shell_log("[shell] init ok");
        osThreadNew(shellTask, NULL, &shellTask_attributes);
    }else{
        shell_log("[shell] init fault");
    }
    
    res = time_us_init();
    if(res == 0){
        shell_log("[timus] init ok");
    }else{
        shell_log("[timus] init fault");
    }

    res = imu_init();
    if(res == 0){
        shell_log("[imu] init ok");
    }else{
        shell_log("[imu] init fault");
    }

	res = move_init();
    if(res == 0){
        shell_log("[move] init ok");
    }else{
        shell_log("[move] init fault");
    }

    collisionTaskHandle = osThreadNew(collisionTask, NULL, &collisionTask_attributes);

    core_init();
    shell_log("[core] init complite, start loop...");
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

void collisionTask( void *args ){
    uint16_t adc_buffer[COLLISION_SENSORS_COUNT];
	HAL_ADC_Start_DMA(&COLLISION_ADC, (uint32_t*)adc_buffer, COLLISION_SENSORS_COUNT);
	HAL_TIM_Base_Start(&COLLISION_TIM_ADC_TRIGGER);
    shell_log("[collision] task started");
    while(1){
        osThreadFlagsWait(1, 0, 1000);
        osThreadFlagsClear(1);
        collision_process(adc_buffer);
    }
}

// hal sensors
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin ){
    move_encoders_process();
}

// collision detection
void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef *hadc ){
    if(hadc->Instance == COLLISION_ADC.Instance){
        //xTaskResumeFromISR()
        // MUST BE REPLACED with xTaskResumeFromISR() or similar!
        osThreadFlagsSet(collisionTaskHandle, 1);
    }
}

void __HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef *htim ){
    move_encoders_overflow_timer_callback(htim);
}