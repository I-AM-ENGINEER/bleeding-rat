// Low level motor driver

#ifndef MOVE_H__
#define MOVE_H__

#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx.h>
#include "motord.h"
#include "encoderd.h"
#include "main.h"

#//define MOVE_DEBUG

#define MOTOR_L_TIM                 htim3
#define MOTOR_R_TIM                 htim3
#define MOTOR_L_PIN1_TIM_CHANNEL    TIM_CHANNEL_1
#define MOTOR_L_PIN2_TIM_CHANNEL    TIM_CHANNEL_2
#define MOTOR_R_PIN1_TIM_CHANNEL    TIM_CHANNEL_3
#define MOTOR_R_PIN2_TIM_CHANNEL    TIM_CHANNEL_4

#define MOTOR_EN_GPIO               MOT_PWR_EN_GPIO_Port
#define MOTOR_EN_PIN                MOT_PWR_EN_Pin



#define ENCODER_L_F_PINA_PORT       NULL
#define ENCODER_L_F_PINA_PIN        0
#define ENCODER_L_F_PINB_PORT       NULL
#define ENCODER_L_F_PINB_PIN        0

#define ENCODER_L_B_PINA_PORT       HALL2_EXTI_GPIO_Port
#define ENCODER_L_B_PINA_PIN        HALL2_EXTI_Pin
#define ENCODER_L_B_PINB_PORT       HALL3_EXTI_GPIO_Port
#define ENCODER_L_B_PINB_PIN        HALL3_EXTI_Pin

#define ENCODER_R_F_PINA_PORT       NULL
#define ENCODER_R_F_PINA_PIN        0
#define ENCODER_R_F_PINB_PORT       NULL
#define ENCODER_R_F_PINB_PIN        0

#define ENCODER_R_B_PINA_PORT       NULL
#define ENCODER_R_B_PINA_PIN        0
#define ENCODER_R_B_PINB_PORT       NULL
#define ENCODER_R_B_PINB_PIN        0


typedef enum{
    ENCODER_FRONT_LEFT,
    ENCODER_FRONT_RIGHT,
    ENCODER_BACK_LEFT,
    ENCODER_BACK_RIGHT,
} encoder_position_t;

typedef struct{
    motord_t motor_l;
    motord_t motor_r;
    encoderd_t encoder_lf;
    encoderd_t encoder_lb;
    encoderd_t encoder_rf;
    encoderd_t encoder_rb;
    bool motion_permision;
} move_t;


int32_t move_init( void );
void move_permit( bool permission );
void move_set_speed( float speed_l, float speed_r );
void move_encoders_process( void );
int32_t move_encoders_get( encoder_position_t encoder_position );

#endif // MOVE_H__
