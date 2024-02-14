#include "motord.h"
#include <math.h>

/**
 * \brief           Set motor PWM duty
 *
 * \param[in]       motord: motor type
 * \param[in]       speed: -1.0...1.0 - new motor speed 
 */
void motord_set_speed( motord_t* motord, float speed ){
    if(speed > 1.0f){
        speed = 1.0f;
    }else if(speed < -1.0f){
        speed = -1.0f;
    }
    
    uint16_t max_value = __HAL_TIM_GET_AUTORELOAD(motord->timer);
    uint16_t compare_value = fabsf(speed) * (float)max_value;
    uint32_t pwm_ch;
    uint32_t break_ch;
    if(speed > 0.0){
        pwm_ch =   motord->pin1_tim_channel;
        break_ch = motord->pin2_tim_channel;
    }else{
        pwm_ch =   motord->pin2_tim_channel;
        break_ch = motord->pin1_tim_channel;
    }

    switch (motord->decay_mode){
        case MOTOR_DECAY_FAST:
            __HAL_TIM_SET_COMPARE(motord->timer, break_ch, UINT16_MAX-1);
            __HAL_TIM_SET_COMPARE(motord->timer, pwm_ch, max_value-compare_value);
            break;
        case MOTOR_DECAY_SLOW:
            __HAL_TIM_SET_COMPARE(motord->timer, break_ch, 0);
            __HAL_TIM_SET_COMPARE(motord->timer, pwm_ch, compare_value);
            break;
        default: break;
    }
}

int32_t motord_init( motord_t* motord, TIM_HandleTypeDef* timer, uint32_t pin1_tim_channel, uint32_t pin2_tim_channel, GPIO_TypeDef* enable_port, uint32_t enable_pin ){
    motord->pin1_tim_channel = pin1_tim_channel;
    motord->pin2_tim_channel = pin2_tim_channel;
    motord->timer = timer;
    motord->enable_port = enable_port;
    motord->enable_pin = enable_pin;
    motord->decay_mode = MOTOR_DECAY_SLOW;
    HAL_TIM_PWM_Start(timer, pin1_tim_channel);
    HAL_TIM_PWM_Start(timer, pin2_tim_channel);
    motord_enable(motord, false);
}

void motord_enable( motord_t* motord, bool enable ){
    if(motord->enable_port == NULL){
        return;
    }
    motord_set_speed(motord, 0.0f);
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
}

void motord_set_decay( motord_t* motord, enum motor_decay_mode_e decay_mode ){
    motord->decay_mode = decay_mode;
}
