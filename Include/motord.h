#ifndef MOTORD_H__
#define MOTORD_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"

#define MOTORD_DEFAULT_DECAY            MOTORD_DECAY_SLOW

#ifndef MOTORD_ENABLE_STATE_INVERSE
#define MOTORD_ENABLE_STATE_INVERSE     true
#endif

typedef enum{
    MOTORD_DECAY_SLOW,
    MOTORD_DECAY_FAST,
} motord_decay_mode_t;

typedef struct{
    TIM_HandleTypeDef* timer;
    uint32_t pin1_tim_channel;
    uint32_t pin2_tim_channel;
    GPIO_TypeDef* enable_port;
    uint16_t      enable_pin;
    float duty;
    motord_decay_mode_t decay_mode;
} motord_t;

/// @brief Иниициализация мотора (структуры motord_t*)
/// @param motord Указатель на структуру мотора
/// @param timer Таймер с 2 каналами в режиме ШИМ выхода
/// @param pin1_tim_channel Канал 1 ШИМ
/// @param pin2_tim_channel Канал 2 ШИМ
/// @param enable_port Порт выхода для включения/выключения двигателя (NULL - без управления вкл/выкл)
/// @param enable_pin Пин выхода для включения/выключения двигателя (если enable_port)
/// @return 0 - успешная инициализация, любое другое значение - ошибка
int32_t motord_init( motord_t* motord, TIM_HandleTypeDef* timer, uint32_t pin1_tim_channel, uint32_t pin2_tim_channel, GPIO_TypeDef* enable_port, uint16_t enable_pin );

/// @brief Установка скважности (напряжения) и направления движения двигателя (РАБОТАЕТ ТОЛЬКО ЕСЛИ ДВИГАТЕЛЬ ВКЛЮЧЕН)
/// @param motord Указатель на структуру мотора
/// @param duty Скважность ШИМ (-1.0...1.0), орицательные значения - движение в обратную сторону
/// @return 0 - успешная установка, любое другое значение - ошибка
int32_t motord_duty_set( motord_t* motord, float duty );

/// @brief Запростить текущую скважность и направление движения двигателя
/// @param motord Указатель на структуру мотора
/// @return Скважность ШИМ (-1.0...1.0), орицательные значения - движение в обратную сторону
float motord_duty_get( motord_t* motord );

/// @brief Переключение вкл/выкл двигателя (не работает, если в motord_init передан аргумент enable_port = NULL)
/// @param motord Указатель на структуру мотора
/// @param enable true - включить, false - выключить
/// @return 0 - успешная установка, любое другое значение - ошибка
int32_t motord_enable( motord_t* motord, bool enable );

/// @brief Установка режима работы драйвера двигателя
/// @param motord Указатель на структуру мотора
/// @param decay_mode Режим работы (MOTORD_DECAY_SLOW или MOTORD_DECAY_FAST)
/// @return 0 - успешная установка, любое другое значение - ошибка
int32_t motord_decay_set( motord_t* motord, motord_decay_mode_t decay_mode );

#endif // MOTORD_H__
