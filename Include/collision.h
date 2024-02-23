#ifndef COLLISION_H__
#define COLLISION_H__

#define COLLISION_DEFAULT_ENGAGE_THRESHOLD          300
#define COLLISION_DEFAULT_DISENGAGE_THRESHOLD       500


#include <stdint.h>
#include <stdbool.h>

typedef enum {
    COLLISION_STATE_NONE,
    COLLISION_STATE_ENGAGE,
    COLLISION_STATE_DISENGAGE,
} collision_sensor_state_t;

typedef struct{
    uint16_t current_value;
    uint16_t last_value;
    uint16_t comp_engage_threshold;
    uint16_t comp_disengage_threshold;
    collision_sensor_state_t comp_state;
} collision_sensor_t;

/// @brief Тип функции обратного вызова для обработки событий датчиков столкновения
/// @param sensor_num Номер датчика, вызвавшего событие
/// @param event_type Тип события столкновения
typedef void (*callback_t)( uint16_t sensor_num, collision_sensor_state_t event_type );

/// @brief Инициализация массива датчиков столкновения
/// @param sensors_array Массив структур датчиков столкновения
/// @param sensors_cnt Количество датчиков в массиве
/// @return 0 в случае успешной инициализации, -1 в случае ошибки
int32_t collision_init( collision_sensor_t *sensors_array, uint16_t sensors_cnt );

/// @brief Обработка новых данных от датчиков столкновения
/// @param new_sensor_data Новые данные с датчиков
/// @param sensor_num Номер датчика, данные которого обрабатываются
/// @return 0 в случае успешной обработки, -1 в случае ошибки
int32_t collision_process( uint16_t new_sensor_data, uint16_t sensor_num );

/// @brief Подключение функции обратного вызова для обработки событий столкновения
/// @param callback Функция обратного вызова для обработки событий
/// @return 0 в случае успешного подключения, -1 в случае ошибки
int32_t collision_attach( callback_t callback );

/// @brief Установка пороговых значений срабатывания и сброса датчика столкновения
/// @param sensor_num Номер датчика, для которого устанавливаются пороги
/// @param engage_threshold Порог срабатывания датчика
/// @param disengage_threshold Порог сброса датчика
/// @return 0 в случае успешной установки, -1 в случае ошибки
int32_t collision_sensor_set_comparator( uint16_t sensor_num, uint16_t engage_threshold, uint16_t disengage_threshold );

/// @brief Получение текущего значения с датчика столкновения
/// @param sensor_num Номер датчика, значение которого запрашивается
/// @return Текущее значение с датчика
uint16_t collision_sensor_get_value( uint16_t sensor_num );

/// @brief Получение текущего состояния датчика столкновения
/// @param sensor_num Номер датчика, состояние которого запрашивается
/// @return Текущее состояние датчика столкновения
collision_sensor_state_t collision_sensor_get_state( uint16_t sensor_num );


#endif // COLLISION_H__
