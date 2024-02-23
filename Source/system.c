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
extern UART_HandleTypeDef huart1;

collision_sensor_t collision_sensors[COLLISION_SENSORS_COUNT];

static volatile uint32_t time_us_high_num = 0;

void time_us_timer_overflow( TIM_HandleTypeDef *htim ){
    time_us_high_num++;
}

inline uint64_t time_us( void ){
    uint32_t time_low = (uint32_t)__HAL_TIM_GET_COUNTER(&SYSTEM_TIM_US);
    #if SYSTEM_TIM_US_32BIT
    uint64_t time_us = (uint64_t)time_low | ((uint64_t)time_us_high_num << 32);
    #else
    uint64_t time_us = (uint64_t)time_low | ((uint64_t)time_us_high_num << 16);
    #endif
    return time_us;
}

inline uint32_t time_ms( void ){
    return time_us()/1000;
}

int32_t time_us_init( void ){
    SYSTEM_TIM_US.PeriodElapsedCallback = time_us_timer_overflow;
    HAL_TIM_Base_Start(&SYSTEM_TIM_US);
    return 0;
}

void delay_us( uint64_t us ){
    uint64_t start_us = time_us();
    if(us > 1000){
        uint32_t ms = us/1000;
        delay(ms);
    }
    while((time_us() - start_us) < us){};
}

inline void delay( uint32_t ms ){
    vTaskDelay(ms);
}

void sys_init( void ){
    int res = 0;
    
    ws2812b_write(100,0,0);

    res = shell_init();
    if(res == 0){
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
        xTaskCreate(task_move, "Move", 1000, NULL, 3000, &task_move_handle);
        shell_log("[move] init ok");
    }else{
        shell_log("[move] init fault");
    }

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
    ws2812b_write(0,100,0);
    vTaskDelay(200);
    ws2812b_write(0,0,10);
    shell_log("[system] start user setup");
    core_init();
    shell_log("[system] user setup complite, start loop");
    while(1){
        core_loop();
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

void HardFault_Handler(void){
    // При сваливании в ошибку, мигаем крамным диодом
    while (1){
		ws2812b_write_block(255, 0, 0);
        for(volatile uint32_t i = 0; i < 1000000; i++){}
		ws2812b_write_block(0, 0, 0);
        for(volatile uint32_t i = 0; i < 1000000; i++){}
    }
}
