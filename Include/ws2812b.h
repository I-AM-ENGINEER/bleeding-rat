#ifndef WS2812B_H__
#define WS2812B_H__


#define WS2812B_PORT    WS2812B_GPIO_Port
#define WS2812B_PIN     WS2812B_Pin
#define WS2812B_TIM     htim10
#define WS2812B_TIM_CH  TIM_CHANNEL_1

#include "main.h"
#include "stm32f4xx.h"
#include <stdint.h>

void ws2812b_write( uint8_t r, uint8_t g, uint8_t b );
void ws2812b_write_block( uint8_t r, uint8_t g, uint8_t b );

#endif // WS2812B_H__