#define _USE_MATH_DEFINES
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "move.h"
#include "shell.h"
#include "main.h"
#include "stm32f4xx.h"

static move_t movement;
static bool move_process_mutex = false;
static float max_power = SERVO_DEFAULT_MAX_POWER;

extern TIM_HandleTypeDef MOTOR_L_TIM;
extern TIM_HandleTypeDef MOTOR_L_TIM;

// Я не ебу, какого хуя это блядина не хочет работать
#if SERVO_MASTER == SERVO_LEFT
#define SERVO_SLAVE SERVO_RIGHT
#else
#define SERVO_SLAVE SERVO_LEFT
#endif


#define MOVE_WHEEL_LENGTH (WHEEL_DIAMETER * 3.14159f)

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

void move_permit( bool permission ){
    #ifdef MOVE_DEBUG
    if(permission){
        shell_log("[move] motors enabled");
    }else{
        shell_log("[move] motors disabled");
    }
    #endif
    motord_enable(&movement.servo_bundle[SERVO_LEFT].motord, permission);
    motord_enable(&movement.servo_bundle[SERVO_RIGHT].motord, permission);
}

void move_power_set( float power_l, float power_r ){
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

void move_speed_set( float speed_l, float speed_r ){
    move_process_mutex = true;
    float rpm_l = move_mmps2rpm(speed_l);
    float rpm_r = move_mmps2rpm(speed_r);
    float ratio_target;

    if((fabsf(rpm_l) < (float)SERVO_MINIMUM_RPM) || (fabsf(rpm_r) < (float)SERVO_MINIMUM_RPM)){
        ratio_target = 0.0f;
        movement.sync_state = MOVE_SYNC_NONE;
    }else{
        if(SERVO_MASTER == SERVO_LEFT){
            ratio_target = rpm_r / rpm_l;
        }else{
            ratio_target = rpm_l / rpm_r;
        }
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
        servo_rpm_set(&movement.servo[SERVO_LEFT],  rpm_l, max_power);
        servo_rpm_set(&movement.servo[SERVO_RIGHT], rpm_r, max_power);
    }
    move_process_mutex = false;
}


void move_position_set( float position_l, float position_r, float max_speed ){
    move_process_mutex = true;
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
        if(SERVO_MASTER == SERVO_LEFT){
            ratio_target = position_difference_r / position_difference_l;
        }else{
            ratio_target = position_difference_l / position_difference_r;
        }
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
        servo_position_set(&movement.servo[SERVO_LEFT],  rotation_l, max_rpm, max_power);
        servo_position_set(&movement.servo[SERVO_RIGHT], rotation_r, max_rpm, max_power);
    }
    move_process_mutex = false;
}

void move_position_reset( void ){
    move_process_mutex = true;
    servo_position_reset(&movement.servo[SERVO_LEFT]);
    servo_position_reset(&movement.servo[SERVO_RIGHT]);
    move_process_mutex = false;
}

void move_pid_speed_config( float kP, float kI, float kD ){
    move_process_mutex = true;
    PIDController_t pid_speed;
    PIDController_Init(&pid_speed);
    pid_speed.Kp        = SERVO_DEFAULT_PID_SPEED_KP;
    pid_speed.Ki        = SERVO_DEFAULT_PID_SPEED_KI;
    pid_speed.Kd        = SERVO_DEFAULT_PID_SPEED_KD;
    pid_speed.tau       = SERVO_DEFAULT_PID_SPEED_TAU;
    pid_speed.limMin    = -1.0f;
    pid_speed.limMax    =  1.0f;
    pid_speed.limMinInt = -1.0f;
    pid_speed.limMaxInt =  1.0f;
    pid_speed.T = 0.001f;
    movement.servo_bundle[SERVO_LEFT].pid_speed  = pid_speed;
    movement.servo_bundle[SERVO_RIGHT].pid_speed = pid_speed;
    move_process_mutex = false;
}

void move_pid_position_config( float kP, float kI, float kD ){
    move_process_mutex = true;
    PIDController_t pid_position;
    PIDController_Init(&pid_position);
    pid_position.Kp         = SERVO_DEFAULT_PID_POSITION_KP;
    pid_position.Ki         = SERVO_DEFAULT_PID_POSITION_KI;
    pid_position.Kd         = SERVO_DEFAULT_PID_POSITION_KD;
    pid_position.tau        = SERVO_DEFAULT_PID_POSITION_TAU;
    pid_position.limMin     = -SERVO_MAXIMUM_RPM;
    pid_position.limMax     =  SERVO_MAXIMUM_RPM;
    pid_position.limMinInt  = -SERVO_MAXIMUM_RPM;
    pid_position.limMaxInt  =  SERVO_MAXIMUM_RPM;
    pid_position.T = 0.001f;
    movement.servo_bundle[SERVO_LEFT].pid_distance  = pid_position;
    movement.servo_bundle[SERVO_RIGHT].pid_distance = pid_position;
    move_process_mutex = false;
}

void move_pid_sync_config( float kP, float kI, float kD ){
    move_process_mutex = true;
    PIDController_t pid_sync;
    PIDController_Init(&pid_sync);
    pid_sync.Kp         = SERVO_DEFAULT_PID_SYNC_KP;
    pid_sync.Ki         = SERVO_DEFAULT_PID_SYNC_KI;
    pid_sync.Kd         = SERVO_DEFAULT_PID_SYNC_KD;
    pid_sync.tau        = SERVO_DEFAULT_PID_SYNC_TAU;
    pid_sync.limMin     = -SERVO_MAXIMUM_RPM;
    pid_sync.limMax     =  SERVO_MAXIMUM_RPM;
    pid_sync.limMinInt  = -SERVO_MAXIMUM_RPM;
    pid_sync.limMaxInt  =  SERVO_MAXIMUM_RPM;
    pid_sync.T = 0.001f;
    movement.pid_distance_sync = pid_sync;
    move_process_mutex = false;
}

void move_max_power_set( float power ){
    if(power > 1.0f){
        max_power = 1.0f;
    }else if(power < 0.0f){
        max_power = 0.0f;
    }else{
        max_power = power;
    }
}

void move_process( void ){
    if(move_process_mutex){
        return;
    }

    float rotation_m = servo_position_get(&movement.servo[SERVO_MASTER]);
    float rotation_s = servo_position_get(&movement.servo[SERVO_SLAVE]);
    
    float rotation_start_m = movement.servo_bundle[SERVO_MASTER].rotation_start;
    float rotation_start_s = movement.servo_bundle[SERVO_SLAVE].rotation_start;

    float rotation_difference_m = rotation_start_m - rotation_m;
    float rotation_difference_s = rotation_start_s - rotation_s;


    if(movement.sync_state != MOVE_SYNC_NONE){
        float ratio_target = movement.sync_ratio_target;
        float ratio_current;
        if(fabsf(rotation_difference_m) < 0.1){
            ratio_current = 1.0f;
        }else{
            ratio_current = rotation_difference_s / rotation_difference_m;
        }

        float virtual_difference_current = rotation_difference_m * ratio_current;
        float virtual_difference_target  = rotation_difference_m * ratio_target;
        float virtual_difference_error = virtual_difference_current - virtual_difference_target;

        float servo_master_rpm_target = movement.servo_bundle[SERVO_MASTER].rpm_target;

        float servo_slave_rpm_offset = PIDController_Update(&movement.pid_distance_sync, 0.0f, virtual_difference_error);
        float servo_slave_rpm_target = movement.servo_bundle[SERVO_SLAVE].rpm_target;
        
        if(virtual_difference_target > 0.0f){
            servo_slave_rpm_offset = -servo_slave_rpm_offset;
        }
        
        float servo_slave_rpm_new = servo_slave_rpm_target - servo_slave_rpm_offset;

        if(movement.sync_state == MOVE_SYNC_SPEED){
            servo_rpm_set(&movement.servo[SERVO_SLAVE], servo_slave_rpm_new, max_power);
            servo_rpm_set(&movement.servo[SERVO_MASTER], servo_master_rpm_target, max_power);
        }

        if(movement.sync_state == MOVE_SYNC_POSITION){
            float servo_master_position_target = movement.servo_bundle[SERVO_MASTER].rotation_target;
            float servo_slave_position_target  = movement.servo_bundle[SERVO_SLAVE].rotation_target;
            servo_position_set(&movement.servo[SERVO_SLAVE], servo_slave_position_target, servo_slave_rpm_new, max_power);
            servo_position_set(&movement.servo[SERVO_MASTER], servo_master_position_target, servo_master_rpm_target, max_power);
        }
    }
    
    servo_process(&movement.servo[SERVO_MASTER]);
    servo_process(&movement.servo[SERVO_SLAVE]);
}

int32_t move_init( void ){
    int32_t res = 0;
    int32_t tmp_res = 0;

    // Инициализация двигателей
    tmp_res = motord_init(&movement.servo_bundle[SERVO_LEFT].motord, &MOTOR_L_TIM, MOTOR_L_PIN2_TIM_CHANNEL, MOTOR_L_PIN1_TIM_CHANNEL, MOTOR_EN_GPIO, MOTOR_EN_PIN);
    if(tmp_res != 0){
        res = 1;
        shell_log("[move] motor left init fault");
    }else{
        motord_decay_set(&movement.servo_bundle[SERVO_LEFT].motord, MOTOR_DEFAULT_DECAY);
    }

    tmp_res = motord_init(&movement.servo_bundle[SERVO_RIGHT].motord, &MOTOR_R_TIM, MOTOR_R_PIN2_TIM_CHANNEL, MOTOR_R_PIN1_TIM_CHANNEL, MOTOR_EN_GPIO, MOTOR_EN_PIN);
    if(tmp_res != 0){
        res = -1;
        shell_log("[move] motor right init fault");
    }else{
        motord_decay_set(&movement.servo_bundle[SERVO_RIGHT].motord, MOTOR_DEFAULT_DECAY);
    }
    
    // Инициализация энкодеров
    tmp_res = encoderd_init(&movement.servo_bundle[SERVO_LEFT].encoderd,\
        (float)SERVO_MINIMUM_RPM/(float)ENCODER_STEPS_IN_ROTATION,\
        ENCODER_L_PINA_PORT, ENCODER_L_PINA_PIN,\
        ENCODER_L_PINB_PORT, ENCODER_L_PINB_PIN
    );
    if(tmp_res != 0){
        res = -1;
        shell_log("[move] encoder left init fault");
    }

    tmp_res = encoderd_init(&movement.servo_bundle[SERVO_RIGHT].encoderd,\
        (float)SERVO_MINIMUM_RPM/(float)ENCODER_STEPS_IN_ROTATION,\
        ENCODER_R_PINA_PORT, ENCODER_R_PINA_PIN,\
        ENCODER_R_PINB_PORT, ENCODER_R_PINB_PIN
    );
    if(tmp_res != 0){
        res = -1;
        shell_log("[move] encoder right init fault");
    }
    
    // Инициализация ПИД серво (ООС по скорости)
    move_pid_speed_config( SERVO_DEFAULT_PID_SPEED_KP, SERVO_DEFAULT_PID_SPEED_KI, SERVO_DEFAULT_PID_SPEED_KD );
    // Инициализация ПИД серво (ООС по положению)
    move_pid_position_config( SERVO_DEFAULT_PID_POSITION_KP, SERVO_DEFAULT_PID_POSITION_KI, SERVO_DEFAULT_PID_POSITION_KD );

    // Инициалиизация серводвигателей
    tmp_res = servo_init(&movement.servo[SERVO_LEFT],\
        &movement.servo_bundle[SERVO_LEFT].motord,\
        &movement.servo_bundle[SERVO_LEFT].encoderd, 
        ENCODER_STEPS_IN_ROTATION, SERVO_MINIMUM_RPM,\
        &movement.servo_bundle[SERVO_LEFT].pid_speed,\
        &movement.servo_bundle[SERVO_LEFT].pid_distance
    );
    if(tmp_res != 0){
        res = -1;
        shell_log("[move] servo left init fault");
    }

    tmp_res = servo_init(&movement.servo[SERVO_RIGHT],\
        &movement.servo_bundle[SERVO_RIGHT].motord,\
        &movement.servo_bundle[SERVO_RIGHT].encoderd,\
        ENCODER_STEPS_IN_ROTATION, SERVO_MINIMUM_RPM,\
        &movement.servo_bundle[SERVO_RIGHT].pid_speed,\
        &movement.servo_bundle[SERVO_RIGHT].pid_distance
    );
    if(tmp_res != 0){
        res = -1;
        shell_log("[move] servo right init fault");
    }

    // Инициализация ПИД синхронизация серводвигателей
    move_pid_sync_config( SERVO_DEFAULT_PID_SYNC_KP, SERVO_DEFAULT_PID_SYNC_KI, SERVO_DEFAULT_PID_SYNC_KD );
    
    return res;
}

void move_encoders_process( void ){
    int32_t res = 0;
    res = encoderd_process_isr(&movement.servo_bundle[SERVO_LEFT].encoderd);
    if(res == -2){
        shell_log("[move] encoder left skip step");
    }

    res = encoderd_process_isr(&movement.servo_bundle[SERVO_RIGHT].encoderd);
    if(res == -2){
        shell_log("[move] encoder right skip step");
    }
}
