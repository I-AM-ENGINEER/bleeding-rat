#ifndef SYSTEM_H__
#define SYSTEM_H__

#define SYSTEM_TIM_US               htim7
#define COLLISION_TIM_ADC_TRIGGER   htim8
#define COLLISION_ADC               hadc1
#define COLLISION_SENSORS_COUNT     5

#include <stdint.h>

uint64_t time_us( void );
uint32_t time_ms( void );
int32_t time_us_init( void );
void delay_us( uint32_t us );
void delay( uint32_t ms );
void sys_init( void );

// This is fix for HAL_TIM_PeriodElapsedCallback defined in main.c
void __HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef *htim );

#endif // SYSTEM_H__
