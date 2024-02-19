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

#if SERVO_MASTER == SERVO_LEFT
#define SERVO_SLAVE SERVO_RIGHT
#else
#define SERVO_SLAVE SERVO_LEFT
#endif

#define MOVE_WHEEL_LENGTH (WHEEL_DIAMETER * 3.14159f)

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

static inline float move_rotate2mm( float rotate ){
    float mm = rotate * MOVE_WHEEL_LENGTH;
    return mm;
}

static inline float move_mm2rotate( float mm ){
    float rotate = mm / MOVE_WHEEL_LENGTH;
    return rotate;
}

static inline float move_rpm2mmps( float rpm ){
    float mmps = rpm * (MOVE_WHEEL_LENGTH / 60.0f);
    return mmps;
}

static inline float move_mmps2rpm( float mmps ){
    float rpm = mmps * (60.0f / MOVE_WHEEL_LENGTH);
    return rpm;
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
    movement.sync_state = MOVE_SYNC_NONE;
    servo_mode_set(&movement.servo[SERVO_LEFT], SERVO_MODE_NO_FEEDBACK);
    servo_mode_set(&movement.servo[SERVO_RIGHT], SERVO_MODE_NO_FEEDBACK);
    servo_power_set(&movement.servo[SERVO_LEFT], power_l);
    servo_power_set(&movement.servo[SERVO_RIGHT], power_r);
}

void move_servos_speed_set( float speed_l, float speed_r ){
    float rpm_l = move_mmps2rpm(speed_l);
    float rpm_r = move_mmps2rpm(speed_r);
    float ratio_target;

    if((fabsf(rpm_l) < (float)SERVO_MINIMUM_RPM) || (fabsf(rpm_r) < (float)SERVO_MINIMUM_RPM)){
        ratio_target = 0.0f;
        movement.sync_state = MOVE_SYNC_NONE;
    }else{
        ratio_target = rpm_r / rpm_l;
        movement.sync_state = MOVE_SYNC_SPEED;
    }
    
    movement.sync_ratio_target = ratio_target;
    
    movement.servo_bundle[SERVO_LEFT].rotation_start  = servo_position_get(&movement.servo[SERVO_LEFT]);
    movement.servo_bundle[SERVO_RIGHT].rotation_start = servo_position_get(&movement.servo[SERVO_RIGHT]);

    movement.servo_bundle[SERVO_LEFT].rpm_target  = rpm_l;
    movement.servo_bundle[SERVO_RIGHT].rpm_target = rpm_r;

    // Переход в режим ОС по скорости
    servo_mode_set(&movement.servo[SERVO_LEFT],  SERVO_MODE_SPEED_FEEDBACK);
    servo_mode_set(&movement.servo[SERVO_RIGHT], SERVO_MODE_SPEED_FEEDBACK);

    // Если синхронизации нет, запуск двигателей без неё
    if(movement.sync_state == MOVE_SYNC_NONE){
        servo_rpm_set(&movement.servo[SERVO_LEFT],  rpm_l, SERVO_MAX_POWER);
        servo_rpm_set(&movement.servo[SERVO_RIGHT], rpm_r, SERVO_MAX_POWER);
    }
}


void move_servos_position_set( float position_l, float position_r, float max_speed ){
    float max_rpm = move_mmps2rpm(max_speed);

    float rotation_l = move_mm2rotate(position_l);
    float rotation_r = move_mm2rotate(position_r);

    float rotation_current_l = servo_position_get(&movement.servo[SERVO_LEFT]);
    float rotation_current_r = servo_position_get(&movement.servo[SERVO_RIGHT]);
    
    float position_current_l = move_rotate2mm(rotation_current_l);
    float position_current_r = move_rotate2mm(rotation_current_r);

    float position_difference_l = position_current_l - position_l;
    float position_difference_r = position_current_r - position_r;

    float ratio_target;

    if((fabsf(max_rpm) < (float)SERVO_MINIMUM_RPM) ||\
        (fabsf(position_difference_l) < (float)SERVO_MINIMUM_DISTANCE) ||\
        (fabsf(position_difference_r) < (float)SERVO_MINIMUM_DISTANCE)\
    ){
        ratio_target = 0.0f;
        movement.sync_state = MOVE_SYNC_NONE;
    }else{
        ratio_target = position_difference_r / position_difference_l;
        movement.sync_state = MOVE_SYNC_POSITION;
    }

    movement.sync_ratio_target = ratio_target;
    
    movement.servo_bundle[SERVO_LEFT].rotation_start  = rotation_current_l;
    movement.servo_bundle[SERVO_RIGHT].rotation_start = rotation_current_r;

    movement.servo_bundle[SERVO_LEFT].rpm_target  = max_rpm;
    movement.servo_bundle[SERVO_RIGHT].rpm_target = max_rpm;

    movement.servo_bundle[SERVO_LEFT].rotation_target  = rotation_l;
    movement.servo_bundle[SERVO_RIGHT].rotation_target = rotation_r;

    // Переход в режим ОС по положению
    servo_mode_set(&movement.servo[SERVO_LEFT],  SERVO_MODE_POSITION_FEEDBACK);
    servo_mode_set(&movement.servo[SERVO_RIGHT], SERVO_MODE_POSITION_FEEDBACK);

    // Если синхронизации нет, запуск двигателей без неё
    if(movement.sync_state == MOVE_SYNC_NONE){
        servo_position_set(&movement.servo[SERVO_LEFT],  rotation_l, max_rpm, SERVO_MAX_POWER);
        servo_position_set(&movement.servo[SERVO_RIGHT], rotation_r, max_rpm, SERVO_MAX_POWER);
    }
}

void move_servos_position_reset( void ){
    servo_position_reset(&movement.servo[SERVO_LEFT]);
    servo_position_reset(&movement.servo[SERVO_RIGHT]);
}

servo_status_t move_servos_status_get( void ){
    return SERVO_STATUS_DISABLES;
}

void move_servos_process( void ){
    float rotation_l = servo_position_get(&movement.servo[SERVO_LEFT]);
    float rotation_r = servo_position_get(&movement.servo[SERVO_RIGHT]);

    float rotation_start_l = movement.servo_bundle[SERVO_LEFT].rotation_start;
    float rotation_start_r = movement.servo_bundle[SERVO_RIGHT].rotation_start;

    float rotation_difference_l = rotation_start_l - rotation_l;
    float rotation_difference_r = rotation_start_r - rotation_r;


    if(movement.sync_state != MOVE_SYNC_NONE){
        float ratio_target = movement.sync_ratio_target;
        float ratio_current = rotation_difference_r / rotation_difference_l;
        
        
        float virtual_difference_current = rotation_difference_l * ratio_current;
        float virtual_difference_target  = rotation_difference_l * ratio_target;
        float virtual_difference_error = virtual_difference_current - virtual_difference_target;

        float servo_master_rpm_target = movement.servo_bundle[SERVO_MASTER].rpm_target;

        float servo_slave_rpm_offset = PIDController_Update(&movement.pid_distance_sync, 0.0f, virtual_difference_error);
        float servo_slave_rpm_target = movement.servo_bundle[SERVO_SLAVE].rpm_target;
        float servo_slave_rpm_new = servo_slave_rpm_target + servo_slave_rpm_offset;

        if(movement.sync_state == MOVE_SYNC_SPEED){
            servo_rpm_set(&movement.servo[SERVO_SLAVE], servo_slave_rpm_new, SERVO_MAX_POWER);
            servo_rpm_set(&movement.servo[SERVO_MASTER], servo_master_rpm_target, SERVO_MAX_POWER);
        }

        if(movement.sync_state == MOVE_SYNC_POSITION){
            float servo_master_position_target = movement.servo_bundle[SERVO_MASTER].rotation_target;
            float servo_slave_position_target  = movement.servo_bundle[SERVO_SLAVE].rotation_target;

            servo_position_set(&movement.servo[SERVO_SLAVE], servo_slave_position_target, servo_slave_rpm_new, SERVO_MAX_POWER);
            servo_position_set(&movement.servo[SERVO_MASTER], servo_master_position_target, servo_master_rpm_target, SERVO_MAX_POWER);
        }
    }
    
    servo_process(&movement.servo[SERVO_LEFT]);
    servo_process(&movement.servo[SERVO_RIGHT]);
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
    PIDController_Init(&movement.servo_bundle[SERVO_RIGHT].servo_pid_speed);
    movement.servo_bundle[SERVO_LEFT].servo_pid_speed.Kp = SERVO_DEFAULT_PID_SPEED_KP;
    movement.servo_bundle[SERVO_LEFT].servo_pid_speed.Ki = SERVO_DEFAULT_PID_SPEED_KI;
    movement.servo_bundle[SERVO_LEFT].servo_pid_speed.Kd = SERVO_DEFAULT_PID_SPEED_KD;
    movement.servo_bundle[SERVO_RIGHT].servo_pid_speed = movement.servo_bundle[SERVO_LEFT].servo_pid_speed;


    PIDController_Init(&movement.servo_bundle[SERVO_LEFT].servo_pid_distance);
    PIDController_Init(&movement.servo_bundle[SERVO_RIGHT].servo_pid_distance);
    movement.servo_bundle[SERVO_LEFT].servo_pid_distance.Kp = SERVO_DEFAULT_PID_POSITION_KP;
    movement.servo_bundle[SERVO_LEFT].servo_pid_distance.Ki = SERVO_DEFAULT_PID_POSITION_KI;
    movement.servo_bundle[SERVO_LEFT].servo_pid_distance.Kd = SERVO_DEFAULT_PID_POSITION_KD;
    movement.servo_bundle[SERVO_RIGHT].servo_pid_distance = movement.servo_bundle[SERVO_LEFT].servo_pid_distance;

    PIDController_Init(&movement.pid_distance_sync);
    movement.pid_distance_sync.Kp = SERVO_DEFAULT_PID_SYNC_KP;
    movement.pid_distance_sync.Ki = SERVO_DEFAULT_PID_SYNC_KI;
    movement.pid_distance_sync.Kd = SERVO_DEFAULT_PID_SYNC_KD;

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
