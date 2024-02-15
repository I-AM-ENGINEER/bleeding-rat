#ifndef SYSTEM_H__
#define SYSTEM_H__

#define SYSTEM_TIM_US               htim7
#define COLLISION_TIM_ADC_TRIGGER   htim8
#define COLLISION_ADC               hadc1

#include <stdint.h>

uint64_t time_us( void );
uint32_t time_ms( void );
int32_t time_us_init( void );
void delay_us( uint32_t us );
void delay( uint32_t ms );
void sys_init( void );

#endif // SYSTEM_H__
