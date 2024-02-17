#include "motord.h"
#include <math.h>
 
int32_t motord_duty_set( motord_t* motord, float duty ){
    if(motord == NULL){
        return -1;
    }
    if(duty > 1.0f){
        duty = 1.0f;
    }else if(duty < -1.0f){
        duty = -1.0f;
    }
    
    motord->duty = duty;
    uint16_t max_value = __HAL_TIM_GET_AUTORELOAD(motord->timer);
    uint16_t compare_value = fabsf(duty) * (float)max_value;
    uint32_t pwm_ch;
    uint32_t break_ch;
    if(duty > 0.0){
        pwm_ch =   motord->pin1_tim_channel;
        break_ch = motord->pin2_tim_channel;
    }else{
        pwm_ch =   motord->pin2_tim_channel;
        break_ch = motord->pin1_tim_channel;
    }

    switch (motord->decay_mode){
        case MOTORD_DECAY_FAST:
            __HAL_TIM_SET_COMPARE(motord->timer, break_ch, UINT16_MAX-1);
            __HAL_TIM_SET_COMPARE(motord->timer, pwm_ch, max_value-compare_value);
            break;
        case MOTORD_DECAY_SLOW:
            __HAL_TIM_SET_COMPARE(motord->timer, break_ch, 0);
            __HAL_TIM_SET_COMPARE(motord->timer, pwm_ch, compare_value);
            break;
        default: break;
    }

    return -1;
}

float motord_duty_get( motord_t* motord ){
    if(motord == NULL){
        return 0.0f;
    }

    return motord->duty;
}

int32_t motord_enable( motord_t* motord, bool enable ){
    if(motord == NULL){
        return -1;
    }
    if(motord->enable_port == NULL){
        return -2;
    }

    motord_duty_set(motord, 0.0f);
    #if MOTORD_ENABLE_STATE_INVERSE
    if(enable){
        HAL_GPIO_WritePin(motord->enable_port, motord->enable_pin, GPIO_PIN_SET);
    }else{
        HAL_GPIO_WritePin(motord->enable_port, motord->enable_pin, GPIO_PIN_RESET);
    }
    #else
    if(enable){
        HAL_GPIO_WritePin(motord->enable_port, motord->enable_pin, GPIO_PIN_RESET);
    }else{
        HAL_GPIO_WritePin(motord->enable_port, motord->enable_pin, GPIO_PIN_SET);
    }
    #endif

    return 0;
}

int32_t motord_decay_set( motord_t* motord, motord_decay_mode_t decay_mode ){
    if(motord == NULL){
        return -1;
    }

    motord->decay_mode = decay_mode;
    return 0;
}

int32_t motord_init( motord_t* motord, TIM_HandleTypeDef* timer, uint32_t pin1_tim_channel, uint32_t pin2_tim_channel, GPIO_TypeDef* enable_port, uint16_t enable_pin ){
    if(motord == NULL){
        return -1;
    }
    if(timer == NULL){
        return -2;
    }
    if(enable_port == NULL){
        return -3;
    }
    motord->pin1_tim_channel = pin1_tim_channel;
    motord->pin2_tim_channel = pin2_tim_channel;
    motord->timer = timer;
    motord->enable_port = enable_port;
    motord->enable_pin = enable_pin;
    motord->decay_mode = MOTORD_DEFAULT_DECAY;

    motord_enable(motord, false);
    HAL_TIM_PWM_Start(timer, pin1_tim_channel);
    HAL_TIM_PWM_Start(timer, pin2_tim_channel);

    return 0;
}
