#include "stm32f4xx.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "shell.h"
#include "core.h"
#include "move.h"
#include "imu.h"
#include "collision.h"

TaskHandle_t task_app_handle;
TaskHandle_t task_collision_handle;
TaskHandle_t task_shell_handle;
TaskHandle_t task_move_handle;
TaskHandle_t task_imu_handle;


void task_app( void *args );
void task_collision( void *args );
void task_shell( void *args );
void task_move( void *args );
void task_imu( void *args );


extern TIM_HandleTypeDef SYSTEM_TIM_US;
extern TIM_HandleTypeDef COLLISION_TIM_ADC_TRIGGER;
extern ADC_HandleTypeDef COLLISION_ADC;

collision_sensor_t collision_sensors[COLLISION_SENSORS_COUNT];

inline uint64_t time_us( void ){
    return ((uint64_t)time_ms() * 1000) + (uint64_t)__HAL_TIM_GET_COUNTER(&SYSTEM_TIM_US);
}

inline uint32_t time_ms( void ){
    return xTaskGetTickCount() * (1000/configTICK_RATE_HZ);
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
    vTaskDelay(ms);
}

void sys_init( void ){
    int res = 0;
    res = shell_init();
    if(res == 0){
        //xTaskCreate(task_shell, "Shell", 300, NULL, 100, &task_shell_handle);
        shell_log("[shell] init ok");
    }else{
        shell_log("[shell] init fault");
    }

    res = time_us_init();
    if(res == 0){
        shell_log("[timus] init ok");
    }else{
        shell_log("[timus] init fault");
    }
    
	res = move_init();
    if(res == 0){
        shell_log("[move] init ok");
    }else{
        shell_log("[move] init fault");
    }
    // WARNING! Task started anyway, results of init ignored
    xTaskCreate(task_move, "Move", 1000, NULL, 3000, &task_move_handle);

    res = collision_init(collision_sensors, COLLISION_SENSORS_COUNT);
    if(res == 0){
        xTaskCreate(task_collision, "Collision", 1000, NULL, 5000, &task_collision_handle);
        shell_log("[collision] init ok");
    }else{
        shell_log("[collision] init fault");
    }
    
    //xTaskCreate(task_imu, "IMU", 1000, NULL, 3000, &task_imu_handle);
    xTaskCreate(task_app, "User app", 1000, NULL, 1000, &task_app_handle);
    vTaskStartScheduler();
    

    while (1);
}

void task_imu( void *args ){
    uint32_t res = imu_init();
    if(res == 0){
        shell_log("[imu] init ok");
    }else{
        shell_log("[imu] init fault");
        vTaskDelete(task_imu_handle);
    }
    while (1){
        vTaskDelay(10);
    }
}

void task_app( void *args ){
    UNUSED(args);
    shell_log("[system] start user setup");
    core_init();
    shell_log("[system] user setup complite, start loop");
    while(1){
        core_loop();
    }
}

void task_shell( void *args ){
    while(1){
        //shell_process();
        vTaskDelay(10);
    }
}

static uint16_t adc_buffer[COLLISION_SENSORS_COUNT];
void task_collision( void *args ){
	HAL_TIM_Base_Start(&COLLISION_TIM_ADC_TRIGGER);
	HAL_ADC_Start_DMA(&COLLISION_ADC, (uint32_t*)adc_buffer, COLLISION_SENSORS_COUNT);
    while(1){
        if(ulTaskNotifyTake(pdTRUE, 100) == 0){
            HAL_ADC_Start_DMA(&COLLISION_ADC, (uint32_t*)adc_buffer, COLLISION_SENSORS_COUNT);
            HAL_TIM_Base_Start(&COLLISION_TIM_ADC_TRIGGER);
            shell_log("[collision] DMA unknown error");
            continue;
        }
        for(uint32_t i = 0; i < COLLISION_SENSORS_COUNT; i++){
            collision_process(adc_buffer[i], i);
        }
    }
}

void task_move( void *args ){
    while (1){
        move_process();
        vTaskDelay(1);
    }
}

// hal sensors
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin ){
    move_encoders_process();
}

// collision detection
void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef *hadc ){
    if(hadc->Instance == COLLISION_ADC.Instance){
        BaseType_t xHigherPriorityTaskWoken  = pdFALSE;
        vTaskNotifyGiveFromISR(task_collision_handle, &xHigherPriorityTaskWoken );
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
