// Low level motor driver

#ifndef MOVE_H__
#define MOVE_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "encoderd.h"
#include "motord.h"
#include "servo.h"
#include "PID.h"
#include "main.h"

#define SERVO_COUNT                 2
#define SERVO_MAX_POWER             1.0f                
#define SERVO_MASTER                SERVO_LEFT          // Ведущий сервопривод, ведомый будет подстраиваться под него
#define ENCODERS_COUNT              4
#define MOTOR_DEFAULT_DECAY         MOTORD_DECAY_FAST
#define SERVO_MINIMUM_RPM           30.0f
#define SERVO_MINIMUM_DISTANCE      1.0f
#define WHEEL_DIAMETER              12.0f
#define ENCODER_STEPS_IN_ROTATION   12

#define SERVO_DEFAULT_PID_SPEED_KP          1.0f
#define SERVO_DEFAULT_PID_SPEED_KI          1.0f
#define SERVO_DEFAULT_PID_SPEED_KD          1.0f

#define SERVO_DEFAULT_PID_POSITION_KP       1.0f
#define SERVO_DEFAULT_PID_POSITION_KI       1.0f
#define SERVO_DEFAULT_PID_POSITION_KD       1.0f

#define SERVO_DEFAULT_PID_SYNC_KP           1.0f
#define SERVO_DEFAULT_PID_SYNC_KI           1.0f
#define SERVO_DEFAULT_PID_SYNC_KD           1.0f

#//define MOVE_DEBUG

#define MOTOR_L_TIM                 htim3
#define MOTOR_R_TIM                 htim3
#define MOTOR_L_PIN1_TIM_CHANNEL    TIM_CHANNEL_2
#define MOTOR_L_PIN2_TIM_CHANNEL    TIM_CHANNEL_1
#define MOTOR_R_PIN1_TIM_CHANNEL    TIM_CHANNEL_3
#define MOTOR_R_PIN2_TIM_CHANNEL    TIM_CHANNEL_4

#define MOTOR_EN_GPIO               MOT_PWR_EN_GPIO_Port
#define MOTOR_EN_PIN                MOT_PWR_EN_Pin


#define ENCODER_L_F_PINA_PORT       NULL
#define ENCODER_L_F_PINA_PIN        0
#define ENCODER_L_F_PINB_PORT       NULL
#define ENCODER_L_F_PINB_PIN        0
//#define ENCODER_L_F_PERIOD_TIM      NULL

#define ENCODER_L_B_PINA_PORT       HALL3_EXTI_GPIO_Port
#define ENCODER_L_B_PINA_PIN        HALL3_EXTI_Pin
#define ENCODER_L_B_PINB_PORT       HALL2_EXTI_GPIO_Port
#define ENCODER_L_B_PINB_PIN        HALL2_EXTI_Pin
#define ENCODER_L_B_PERIOD_TIM      htim13

#define ENCODER_R_F_PINA_PORT       NULL
#define ENCODER_R_F_PINA_PIN        0
#define ENCODER_R_F_PINB_PORT       NULL
#define ENCODER_R_F_PINB_PIN        0
//#define ENCODER_R_F_PERIOD_TIM      NULL

#define ENCODER_R_B_PINA_PORT       HALL4_EXTI_GPIO_Port
#define ENCODER_R_B_PINA_PIN        HALL4_EXTI_Pin
#define ENCODER_R_B_PINB_PORT       HALL1_EXTI_GPIO_Port
#define ENCODER_R_B_PINB_PIN        HALL1_EXTI_Pin
#define ENCODER_R_B_PERIOD_TIM      htim14

typedef enum{
    SERVO_STATUS_MOVING,
    SERVO_STATUS_PARKED,
    SERVO_STATUS_DISABLES,
} servo_status_t;

typedef enum{
    MOVE_SYNC_NONE,
    MOVE_SYNC_SPEED,
    MOVE_SYNC_POSITION,
} move_sync_status_t;

typedef enum{
    SERVO_LEFT,
    SERVO_RIGHT,
} servo_position_t;

typedef enum{
    ENCODER_FRONT_LEFT = 0,
    ENCODER_FRONT_RIGHT,
    ENCODER_BACK_LEFT,
    ENCODER_BACK_RIGHT,
} encoder_position_t;

struct servo_bundle_s{
    motord_t servo_motord;
    PIDController_t servo_pid_speed;
    PIDController_t servo_pid_distance;
    float rpm_target;
    float rotation_target;
    float rotation_start;
};

typedef struct{
    servo_t servo[SERVO_COUNT];
    struct servo_bundle_s servo_bundle[SERVO_COUNT];
    encoderd_t encoderd[ENCODERS_COUNT];
    PIDController_t pid_distance_sync;
    move_sync_status_t sync_state;
    float sync_ratio_target;
    bool motion_permision;
} move_t;


int32_t move_init( void );

void move_servos_callback_attach( void (*callback) (void) );
void move_servos_permit( bool permission );
void move_servos_power_set( float power_l, float power_r );
void move_servos_speed_set( float speed_l, float speed_r );
void move_servos_position_set( float position_l, float position_r, float max_speed );
void move_servos_position_reset( void );
servo_status_t move_servos_status_get( void );
servo_t *move_servo_get( servo_position_t servo_position );
encoderd_t *move_encoderd_get( encoder_position_t encoder_position );
// Должно быть 100...1000Гц - обработка ПИД регуляторов двигателей
//void move_servos_process( void );

// Должны вызываться при смене состояния любого пина энкодеров
void move_encoders_process( void );
// Должно вызываться при переполнении таймера любого энкодера
void move_encoders_overflow_timer_irq( TIM_HandleTypeDef *htim );

#endif // MOVE_H__
