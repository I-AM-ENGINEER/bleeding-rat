// Low level motor driver

#ifndef MOVE_H__
#define MOVE_H__

#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx.h>
#include "motord.h"

#define MOVE_DEBUG

#define MOTOR_L_TIM                 htim3
#define MOTOR_R_TIM                 htim3
#define MOTOR_L_PIN1_TIM_CHANNEL    TIM_CHANNEL_1
#define MOTOR_L_PIN2_TIM_CHANNEL    TIM_CHANNEL_2
#define MOTOR_R_PIN1_TIM_CHANNEL    TIM_CHANNEL_3
#define MOTOR_R_PIN2_TIM_CHANNEL    TIM_CHANNEL_4

#define MOTOR_EN_GPIO               MOT_PWR_EN_GPIO_Port
#define MOTOR_EN_PIN                MOT_PWR_EN_Pin

typedef struct{
    motord_t motor_l;
    motord_t motor_r;
    bool motion_permision;
} move_t;



int32_t move_init( void );
void move_permit( bool permission );
void move_set_speed( float speed_l, float speed_r );

#endif // MOVE_H__
