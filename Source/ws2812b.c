#include "ws2812b.h"
#include <stdbool.h>

extern TIM_HandleTypeDef WS2812B_TIM;
static uint32_t cnt = 24;

static uint32_t brg;

void ws2812b_write( uint8_t r, uint8_t g, uint8_t b ){
    cnt = 24;
    brg = ((uint32_t)b << 16) | ((uint32_t)r << 8) | (uint32_t)g;

    if(brg & 0x000001){
        __HAL_TIM_SET_COMPARE(&WS2812B_TIM, WS2812B_TIM_CH, 133);
    }else{
        __HAL_TIM_SET_COMPARE(&WS2812B_TIM, WS2812B_TIM_CH, 66);
    }
    
    __HAL_TIM_SET_AUTORELOAD(&WS2812B_TIM, 209);
    __HAL_TIM_ENABLE_IT(&WS2812B_TIM, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&WS2812B_TIM, TIM_IT_CC1);
    __HAL_TIM_ENABLE(&WS2812B_TIM);

}

//__attribute__((section(".ccmram")))
void TIM1_UP_TIM10_IRQHandler( void ){
    if(__HAL_TIM_GET_FLAG(&WS2812B_TIM, TIM_FLAG_CC1)){
        __HAL_TIM_CLEAR_IT(&WS2812B_TIM, TIM_IT_CC1);
        WS2812B_PORT->BSRR = WS2812B_PIN << 16;
        if(!cnt){
            __HAL_TIM_DISABLE(&WS2812B_TIM);
            __HAL_TIM_CLEAR_IT(&WS2812B_TIM, TIM_IT_UPDATE);
        }
    }
    if (__HAL_TIM_GET_FLAG(&WS2812B_TIM, TIM_FLAG_UPDATE)){
        __HAL_TIM_CLEAR_IT(&WS2812B_TIM, TIM_IT_UPDATE);
        WS2812B_PORT->BSRR = WS2812B_PIN;
        brg >>= 1;
        if(brg & 0x000001){
            __HAL_TIM_SET_COMPARE(&WS2812B_TIM, WS2812B_TIM_CH, 133);
        }else{
            __HAL_TIM_SET_COMPARE(&WS2812B_TIM, WS2812B_TIM_CH, 66);
        }
        cnt--;
    }
}

