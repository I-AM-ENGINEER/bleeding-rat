#ifndef SYSTEM_H__
#define SYSTEM_H__

#define SYSTEM_TIM_US               htim2
#define SYSTEM_TIM_US_32BIT         true
#define COLLISION_TIM_ADC_TRIGGER   htim8
#define COLLISION_ADC               hadc1
#define COLLISION_SENSORS_COUNT     5

#include <stdint.h>

/// @brief Получение текущего времени в микросекундах с момента включения контроллера
/// @return Текущее время в микросекундах
uint64_t time_us( void );

/// @brief Получение текущего времени в миллисекундах с момента включения контроллера
/// @return Текущее время в миллисекундах
uint32_t time_ms( void );

/// @brief Инициализация функции измерения времени в микросекундах
/// @return 0 в случае успешной инициализации, -1 в случае ошибки
int32_t time_us_init( void );

/// @brief Задержка в микросекундах
/// @param us Время задержки в микросекундах
void delay_us( uint64_t us );

/// @brief Задержка в миллисекундах
/// @param ms Время задержки в миллисекундах
void delay( uint32_t ms );

/// @brief Инициализация системы
void sys_init( void );

#endif // SYSTEM_H__
