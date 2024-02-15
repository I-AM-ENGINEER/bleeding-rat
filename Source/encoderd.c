#include "encoderd.h"
#include "shell.h"
#include "string.h"
#include "stdio.h"

const int8_t lookup_table[4][4] = {
    {0, -1, 1, 2},
    {1, 0, 2, -1},
    {-1, 2, 0, 1},
    {2, 1, -1, 0},
};

int encoderd_init( encoderd_t *encoderd,\
    GPIO_TypeDef *pin_a_gpio_port, uint16_t pin_a_pin,\
    GPIO_TypeDef *pin_b_gpio_port, uint16_t pin_b_pin,\
    TIM_HandleTypeDef *htim_period)
{
    if(encoderd == NULL){
        return -1;
    }
    if(pin_a_gpio_port == NULL){
        return -1;
    }
    if(pin_b_gpio_port == NULL){
        return -1;
    }
    encoderd->pin_a_gpio_port = pin_a_gpio_port;
    encoderd->pin_b_gpio_port = pin_b_gpio_port;
    encoderd->pin_a_pin = pin_a_pin;
    encoderd->pin_b_pin = pin_b_pin;
    encoderd->steps = 0;
    encoderd->htim_period = htim_period;
    if(encoderd->htim_period != NULL){
        HAL_TIM_Base_Start_IT(encoderd->htim_period);
    }
    return 0;
}

int32_t encoderd_process( encoderd_t *encoderd ){
    if(encoderd == NULL){
        return -1;
    }
    if(encoderd->pin_a_gpio_port == NULL){
        return -1;
    }
    if(encoderd->pin_b_gpio_port == NULL){
        return -1;
    }
    int32_t res = 0;

    uint8_t pin_a_state = HAL_GPIO_ReadPin(encoderd->pin_a_gpio_port, encoderd->pin_a_pin) == GPIO_PIN_SET ? 1 : 0;
    uint8_t pin_b_state = HAL_GPIO_ReadPin(encoderd->pin_b_gpio_port, encoderd->pin_b_pin) == GPIO_PIN_SET ? 1 : 0;
    uint8_t state = pin_a_state + (pin_b_state << 1);

    int8_t delta = lookup_table[state][encoderd->old_state];

    if(delta == 2){
        res = -2;
    }else if(delta != 0){
        encoderd->steps += (int32_t)delta;
        if((encoderd->htim_period != NULL)){
            encoderd->timer_period = encoderd->htim_period->Instance->CNT;
            encoderd->timer_period_filter[encoderd->timer_period_filter_i++] = encoderd->timer_period;
            if(encoderd->timer_period_filter_i == (sizeof(encoderd->timer_period_filter)/sizeof(*encoderd->timer_period_filter))){
                encoderd->timer_period_filter_i = 0;
            }
            encoderd->htim_period->Instance->CNT = 0;
            if(!(encoderd->htim_period->Instance->CR1 & (TIM_CR1_CEN))){
                __HAL_TIM_ENABLE(encoderd->htim_period);
            }
        }
    }

    encoderd->old_state = state;
    return res;
}

void encoderd_reset( encoderd_t *encoderd ){
    encoderd->steps = 0;
}

int32_t encoderd_get_steps( encoderd_t *encoderd ){
    return encoderd->steps;
}

void encoderd_period_timer_overflow( encoderd_t *encoderd ){
    for(uint16_t i = 0; i < (sizeof(encoderd->timer_period_filter)/sizeof(*encoderd->timer_period_filter)); i++){
        encoderd->timer_period_filter[i] = UINT16_MAX;
    }
    __HAL_TIM_DISABLE(encoderd->htim_period);
    encoderd->htim_period->Instance->CNT = UINT16_MAX;
}

float encoderd_get_steps_per_second( encoderd_t *encoderd ){
    float sps = 0.0f;
    uint32_t period = 0;
    for(uint32_t i = 0; i < (sizeof(encoderd->timer_period_filter)/sizeof(*encoderd->timer_period_filter)); i++){
        period += encoderd->timer_period_filter[i];
    }
    period /= sizeof(encoderd->timer_period_filter)/sizeof(*encoderd->timer_period_filter);
    if(period != UINT16_MAX){
        uint32_t timer_clock = HAL_RCC_GetPCLK1Freq() * 2;
        uint32_t timer_period_ticks = (uint32_t)(encoderd->htim_period->Instance->PSC+1) * period;
        sps = (float)timer_clock / (float)timer_period_ticks;
    }
    return sps;
}
