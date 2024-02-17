#define _USE_MATH_DEFINES
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "lwshell/lwshell.h"
#include "move.h"
#include "shell.h"
#include "main.h"
#include "stm32f4xx.h"

static move_t movement;

extern TIM_HandleTypeDef MOTOR_L_TIM;
extern TIM_HandleTypeDef MOTOR_L_TIM;

#ifdef ENCODER_L_B_PERIOD_TIM
#define __ENCODER_L_B_PERIOD_TIM &ENCODER_L_B_PERIOD_TIM
extern TIM_HandleTypeDef ENCODER_L_B_PERIOD_TIM;
#else
#define __ENCODER_L_B_PERIOD_TIM NULL
#endif

#ifdef ENCODER_R_B_PERIOD_TIM
#define __ENCODER_R_B_PERIOD_TIM &ENCODER_R_B_PERIOD_TIM
extern TIM_HandleTypeDef ENCODER_R_B_PERIOD_TIM;
#else
#define __ENCODER_R_B_PERIOD_TIM NULL
#endif

#ifdef ENCODER_L_F_PERIOD_TIM
#define __ENCODER_L_F_PERIOD_TIM &ENCODER_L_F_PERIOD_TIM
extern TIM_HandleTypeDef ENCODER_L_F_PERIOD_TIM;
#else
#define __ENCODER_L_F_PERIOD_TIM NULL
#endif

#ifdef ENCODER_R_F_PERIOD_TIM
#define __ENCODER_R_F_PERIOD_TIM &ENCODER_R_F_PERIOD_TIM
extern TIM_HandleTypeDef ENCODER_R_F_PERIOD_TIM;
#else
#define __ENCODER_R_F_PERIOD_TIM NULL
#endif

/* Currently anavailable
int move_cmd_move( int32_t argc, char** argv ){
    for(uint32_t i = 0; i < argc; i++){
        char *argument = argv[i];
        char *next_arg = NULL;
        if((i+1) < argc){
            next_arg = argv[i+1];
        }
        if(!strcmp(argument, "-h")){
            printf("\
                command move help\r\n\
                This command used for control motors manualy\r\n\
                -h,\tprint this help\r\n\
                -sl,\tset speed left motor (-1.0 ... 1.0)\r\n\
                -sr,\tset speed right motor (-1.0 ... 1.0)\r\n\
                -s,\tset speed on both motors (-1.0 ... 1.0)\r\n\
                -b,\tstop motors\r\n\
            ");
        }else if(!strcmp(argument, "-sl")){
            if(next_arg != NULL){
                float new_speed;
                sscanf(next_arg, "%f", &new_speed);
                motord_set_speed(&movement.motord_l, new_speed);
            }
        }else if(!strcmp(argument, "-sr")){
            if(next_arg != NULL){
                float new_speed;
                sscanf(next_arg, "%f", &new_speed);
                motord_set_speed(&movement.motord_r, new_speed);
            }
        }else if(!strcmp(argument, "-s")){
            if(next_arg != NULL){
                float new_speed;
                sscanf(next_arg, "%f", &new_speed);
                motord_set_speed(&movement.motord_l, new_speed);
                motord_set_speed(&movement.motord_r, new_speed);
            }
        }else if(!strcmp(argument, "-b")){
            motord_set_speed(&movement.motord_l, 0);
            motord_set_speed(&movement.motord_r, 0);
        }else{
            printf("unsupported cmd, type \"move -h\" for help\r\n");
            return -1;
        }
    }
    return 0;
}*/

encoderd_t *move_encoderd_get( encoder_position_t encoder_position ){
    encoderd_t *encoderd = NULL;
    if(encoder_position < ENCODERS_COUNT){
        encoderd = &movement.encoderd[encoder_position];
    }
    if(encoderd == NULL){
        shell_log("[move] invalid encoder position");
    }

    return encoderd;
}

servo_t *move_servo_get( servo_position_t servo_position ){
    servo_t *servo = NULL;
    if(servo_position < SERVO_COUNT){
        servo = &movement.servo[servo_position];
    }
    if(servo == NULL){
        shell_log("[move] invalid servo position");
    }

    return servo;
}

float move_encoder_get_rpm( encoder_position_t encoder_position ){
    encoderd_t *encoderd = move_encoderd_get(encoder_position);    
    if(encoderd == NULL){
        return 0.0f;
    }
    if(encoderd->htim_period == NULL){
        shell_log("[move] encoders without timer cant capture rpm/speed");
        return 0.0f;
    }
    float sps = encoderd_get_steps_per_second( encoderd );
    float rps = (sps / (float)ENCODER_STEPS_IN_ROTATION);
    float rpm = rps * 60.0f;

    return rps;
}

float move_rpm2ms( encoder_position_t encoder_position ){
    float rpm = move_encoder_get_rpm( encoder_position );
    float speed_ms = (rpm / 60.0f) * WHEEL_DIAMETER * 2.0f * 3.14159f;
    return speed_ms;
}

void move_servos_permit( bool permission ){
    #ifdef MOVE_DEBUG
    if(permission){
        shell_log("[move] motors enabled");
    }else{
        shell_log("[move] motors disabled");
    }
    #endif
    motord_enable(&movement.servo_bundle[SERVO_LEFT].servo_motord, permission);
    motord_enable(&movement.servo_bundle[SERVO_RIGHT].servo_motord, permission);
}

void move_servos_power_set( float power_l, float power_r ){
    #ifdef MOVE_DEBUG
    shell_log("[move] motors power set: l: %.3f, r: %.3f", \
        power_l\
        power_r\
    );
    #endif
    servo_mode_set(&movement.servo[SERVO_LEFT], SERVO_MODE_NO_FEEDBACK);
    servo_mode_set(&movement.servo[SERVO_RIGHT], SERVO_MODE_NO_FEEDBACK);
    servo_power_set(&movement.servo[SERVO_LEFT], power_l);
    servo_power_set(&movement.servo[SERVO_RIGHT], power_r);
}

void move_servos_speed_set( float speed_l, float speed_r ){
    servo_rpm_set(&movement.servo[SERVO_LEFT], speed_l, SERVO_MAX_POWER);
    servo_rpm_set(&movement.servo[SERVO_RIGHT], speed_r, SERVO_MAX_POWER);
}


void move_servos_position_set( float position_l, float position_r, float max_speed ){
    servo_position_set(&movement.servo[SERVO_LEFT], position_l, max_speed, SERVO_MAX_POWER);
    servo_position_set(&movement.servo[SERVO_RIGHT], position_r, max_speed, SERVO_MAX_POWER);
}

void move_servos_position_reset( void ){
    servo_position_reset(&movement.servo[SERVO_LEFT]);
    servo_position_reset(&movement.servo[SERVO_RIGHT]);
}

servo_status_t move_servos_status_get( void ){
    return SERVO_STATUS_DISABLES;
}

int32_t move_init( void ){
    //lwshell_register_cmd("move", move_cmd_move, "manual motors control");
    int32_t res = 0;
    int32_t tmp_res = 0;

    tmp_res = motord_init(&movement.servo_bundle[SERVO_LEFT].servo_motord, &MOTOR_L_TIM, MOTOR_L_PIN2_TIM_CHANNEL, MOTOR_L_PIN1_TIM_CHANNEL, MOTOR_EN_GPIO, MOTOR_EN_PIN);
    if(tmp_res != 0){
        res = 1;
        shell_log("[move] motor left init fault");
    }else{
        motord_decay_set(&movement.servo_bundle[SERVO_LEFT].servo_motord, MOTOR_DEFAULT_DECAY);
    }

    tmp_res = motord_init(&movement.servo_bundle[SERVO_RIGHT].servo_motord, &MOTOR_R_TIM, MOTOR_R_PIN2_TIM_CHANNEL, MOTOR_R_PIN1_TIM_CHANNEL, MOTOR_EN_GPIO, MOTOR_EN_PIN);
    if(tmp_res != 0){
        res = -1;
        shell_log("[move] motor right init fault");
    }else{
        motord_decay_set(&movement.servo_bundle[SERVO_RIGHT].servo_motord, MOTOR_DEFAULT_DECAY);
    }
    
    tmp_res = encoderd_init(&movement.encoderd[ENCODER_BACK_LEFT],\
        ENCODER_L_B_PINA_PORT, ENCODER_L_B_PINA_PIN,\
        ENCODER_L_B_PINB_PORT, ENCODER_L_B_PINB_PIN,\
        __ENCODER_L_B_PERIOD_TIM
    );
    if(tmp_res != 0){
        res = -1;
        shell_log("[move] encoder left back init fault");
    }

    tmp_res = encoderd_init(&movement.encoderd[ENCODER_FRONT_LEFT],\
        ENCODER_L_F_PINA_PORT, ENCODER_L_F_PINA_PIN,\
        ENCODER_L_F_PINB_PORT, ENCODER_L_F_PINB_PIN,\
        __ENCODER_L_F_PERIOD_TIM
    );
    if(tmp_res != 0){
        res = -1;
        shell_log("[move] encoder left front init fault");
    }

    tmp_res = encoderd_init(&movement.encoderd[ENCODER_BACK_RIGHT],\
        ENCODER_R_B_PINA_PORT, ENCODER_R_B_PINA_PIN,\
        ENCODER_R_B_PINB_PORT, ENCODER_R_B_PINB_PIN,\
        __ENCODER_R_B_PERIOD_TIM
    );
    if(tmp_res != 0){
        res = -1;
        shell_log("[move] encoder right back init fault");
    }

    tmp_res = encoderd_init(&movement.encoderd[ENCODER_FRONT_RIGHT],\
        ENCODER_R_F_PINA_PORT, ENCODER_R_F_PINA_PIN,\
        ENCODER_R_F_PINB_PORT, ENCODER_R_F_PINB_PIN,\
        __ENCODER_R_F_PERIOD_TIM
    );
    if(tmp_res != 0){
        res = -1;
        shell_log("[move] encoder right front init fault");
    }

    PIDController_Init(&movement.servo_bundle[SERVO_LEFT].servo_pid_speed);
    PIDController_Init(&movement.servo_bundle[SERVO_LEFT].servo_pid_distance);

    tmp_res = servo_init(&movement.servo[SERVO_LEFT],\
        &movement.servo_bundle[SERVO_LEFT].servo_motord,\
        &movement.encoderd[ENCODER_BACK_LEFT], ENCODER_STEPS_IN_ROTATION,\
        &movement.servo_bundle[SERVO_LEFT].servo_pid_speed,\
        &movement.servo_bundle[SERVO_LEFT].servo_pid_distance
    );
    if(tmp_res != 0){
        res = -1;
        shell_log("[move] servo left init fault");
    }

    
    PIDController_Init(&movement.servo_bundle[SERVO_RIGHT].servo_pid_speed);
    PIDController_Init(&movement.servo_bundle[SERVO_RIGHT].servo_pid_distance);

    tmp_res = servo_init(&movement.servo[SERVO_RIGHT],\
        &movement.servo_bundle[SERVO_RIGHT].servo_motord,\
        &movement.encoderd[ENCODER_BACK_RIGHT], ENCODER_STEPS_IN_ROTATION,\
        &movement.servo_bundle[SERVO_RIGHT].servo_pid_speed,\
        &movement.servo_bundle[SERVO_RIGHT].servo_pid_distance
    );
    if(tmp_res != 0){
        res = -1;
        shell_log("[move] servo right init fault");
    }

    PIDController_Init(&movement.pid_distance_sync);

    return res;
}

void move_encoders_process( void ){
    int32_t res = 0;
    res = encoderd_process(&movement.encoderd[ENCODER_BACK_LEFT]);
    if(res == -2){
        shell_log("[move] encoder left back skip step");
    }

    res = encoderd_process(&movement.encoderd[ENCODER_BACK_RIGHT]);
    if(res == -2){
        shell_log("[move] encoder left front skip step");
    }

    res = encoderd_process(&movement.encoderd[ENCODER_FRONT_LEFT]);
    if(res == -2){
        shell_log("[move] encoder right back skip step");
    }

    res = encoderd_process(&movement.encoderd[ENCODER_FRONT_RIGHT]);
    if(res == -2){
        shell_log("[move] encoder right front skip step");
    }
}

void move_encoders_overflow_timer_irq( TIM_HandleTypeDef *htim ){
    if(movement.encoderd[ENCODER_BACK_LEFT].htim_period != NULL){
        if(htim->Instance == movement.encoderd[ENCODER_BACK_LEFT].htim_period->Instance){
            encoderd_period_timer_overflow_irq(&movement.encoderd[ENCODER_BACK_LEFT]);
        }
    }
    
    if(movement.encoderd[ENCODER_BACK_RIGHT].htim_period != NULL){
        if(htim->Instance == movement.encoderd[ENCODER_BACK_RIGHT].htim_period->Instance){
            encoderd_period_timer_overflow_irq(&movement.encoderd[ENCODER_BACK_RIGHT]);
        }
    }

    if(movement.encoderd[ENCODER_FRONT_RIGHT].htim_period != NULL){
        if(htim->Instance == movement.encoderd[ENCODER_FRONT_RIGHT].htim_period->Instance){
            encoderd_period_timer_overflow_irq(&movement.encoderd[ENCODER_FRONT_RIGHT]);
        }
    }
    
    if(movement.encoderd[ENCODER_FRONT_LEFT].htim_period != NULL){
        if(htim->Instance == movement.encoderd[ENCODER_FRONT_LEFT].htim_period->Instance){
            encoderd_period_timer_overflow_irq(&movement.encoderd[ENCODER_FRONT_LEFT]);
        }
    }
}
