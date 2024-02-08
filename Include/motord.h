// Low level motor driver

#ifndef MOTORD_H__
#define MOTORD_H__

#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx.h>

#define MOTORD_ENABLE_STATE_INVERSE     true

typedef struct{
    TIM_HandleTypeDef* timer;
    uint32_t pin1_tim_channel;
    uint32_t pin2_tim_channel;
    GPIO_TypeDef* enable_port;
    uint32_t      enable_pin;
    uint32_t pwm_frequency;
    bool break_enable;
} motord_t;

int32_t motord_init( motord_t* motord, TIM_HandleTypeDef* timer, uint32_t pin1_tim_channel, uint32_t pin2_tim_channel, GPIO_TypeDef* enable_port, uint32_t enable_pin );
void motord_set_speed( motord_t* motord, float speed );
void motord_enable( motord_t* motord, bool enable );
void motord_set_break( motord_t* motord, bool active_break );

#endif // MOTORD_H__
