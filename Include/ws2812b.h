#ifndef WS2812B_H__
#define WS2812B_H__


#define WS2812B_PORT    WS2812B_GPIO_Port
#define WS2812B_PIN     WS2812B_Pin
#define WS2812B_TIM     htim10
#define WS2812B_TIM_CH  TIM_CHANNEL_1

#include "main.h"
#include "stm32f4xx.h"
#include <stdint.h>

/// @brief Неблокирующая запись цвета в светодиод WS2812B
/// @param r Красная компонента цвета (от 0 до 255)
/// @param g Зелёная компонента цвета (от 0 до 255)
/// @param b Синяя компонента цвета (от 0 до 255)
void ws2812b_write( uint8_t r, uint8_t g, uint8_t b );

/// @brief Блокирующая запись цвета в светодиод WS2812B (нужная для hardfault)
/// @param r Красная компонента цвета (от 0 до 255)
/// @param g Зелёная компонента цвета (от 0 до 255)
/// @param b Синяя компонента цвета (от 0 до 255)
void ws2812b_write_block( uint8_t r, uint8_t g, uint8_t b );

#endif // WS2812B_H__
