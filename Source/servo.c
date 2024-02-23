#include <math.h>
#include "servo.h"
#include <stdio.h>
#include "shell.h"

int32_t servo_init( servo_t *servo, motord_t *motord, encoderd_t *encoderd, uint32_t steps_per_rotate, float rpm_min, PIDController_t *pid_rpm, PIDController_t *pid_position ){
    if(servo == NULL){
        return -1;
    }
    if(motord == NULL){
        return -2;
    }
    if(encoderd == NULL){
        return -3;
    }

    if(pid_rpm != NULL){
        PIDController_Init(pid_rpm);
    }
    if(pid_position != NULL){
        PIDController_Init(pid_position);
    }
    
    servo->min_rpm = rpm_min;
    servo->motord = motord;
    servo->encoderd = encoderd;
    servo->pid_rpm = pid_rpm;
    servo->pid_position = pid_position;
    servo->steps_per_rotate = steps_per_rotate;
    servo->target_rpm = 0.0f;
    servo->target_postion = 0.0f;
    servo->mode = SERVO_MODE_NO_FEEDBACK;
    
    return 0;
}

int32_t servo_mode_set( servo_t *servo, servo_mode_t mode ){
    if(servo == NULL){
        return -1;
    }
    switch (mode){
        case SERVO_MODE_NO_FEEDBACK: 
            servo->mode = SERVO_MODE_NO_FEEDBACK;
            break;
        case SERVO_MODE_SPEED_FEEDBACK: 
            if(servo->pid_rpm == NULL){
                return -2;
            }

            servo->target_rpm = 0;
            servo->mode = SERVO_MODE_SPEED_FEEDBACK;
            break;
        case SERVO_MODE_POSITION_FEEDBACK: 
            if(servo->pid_rpm == NULL){
                return -2;
            }
            if(servo->pid_position == NULL){
                return -3;
            }

            servo->target_postion = servo_position_get(servo);
            servo->mode = SERVO_MODE_POSITION_FEEDBACK;
            break;
        default:
            return -4;
    }

    return 0;
}

int32_t servo_power_set( servo_t *servo, float power ){
    if(servo == NULL){
        return -1;
    }
    if(servo->mode != SERVO_MODE_NO_FEEDBACK){
        return -2;
    }

    motord_duty_set(servo->motord, power);
    servo->power = power;

    return 0;
}

float servo_power_get( servo_t *servo ){
    if(servo == NULL){
        return 0.0f;
    }
    
    return servo->power;
}

int32_t servo_rpm_set( servo_t *servo, float rpm, float max_power ){
    if(servo == NULL){
        return -1;
    }
    if(servo->mode != SERVO_MODE_SPEED_FEEDBACK){
        return -2;
    }
    if(servo->pid_rpm == NULL){
        return -3;
    }

    servo->target_rpm = rpm;
    servo->max_power = max_power;

    return 0;
}

float servo_rpm_get( servo_t *servo ){
    if(servo == NULL){
        return 0.0f;
    }

    float steps_per_second = encoderd_get_steps_per_second(servo->encoderd);
    float rpm = 60.0f * steps_per_second / (float)servo->steps_per_rotate;

    return rpm;
}

int32_t servo_position_reset( servo_t *servo ){
    if(servo == NULL){
        return -1;
    }

    encoderd_reset(servo->encoderd);

    return 0;
}

int32_t servo_position_set( servo_t *servo, float position, float max_rpm, float max_power ){
    if(servo == NULL){
        return -1;
    }
    if(servo->mode != SERVO_MODE_POSITION_FEEDBACK){
        return -2;
    }
    if(servo->pid_rpm == NULL){
        return -3;
    }
    if(servo->pid_position == NULL){
        return -4;
    }

    servo->target_postion = position;
    servo->target_rpm = max_rpm;
    servo->max_power = max_power;

    return 0;
}

float servo_position_get( servo_t *servo ){
    if(servo == NULL){
        return -1;
    }

    int32_t encoder_step = encoderd_get_steps(servo->encoderd);
    float position = (float)encoder_step/(float)servo->steps_per_rotate;

    return position;
}

int32_t servo_process( servo_t *servo ){
    int32_t res = 0;
    encoderd_process_rpm(servo->encoderd);
    float rpm = servo_rpm_get(servo);
    float position = servo_position_get(servo);

    // Если без обратной связи, ничего не рассчитываем
    if((servo->pid_rpm != NULL) && (servo->mode != SERVO_MODE_NO_FEEDBACK)){
        float targer_rpm = servo->target_rpm;
        if((servo->pid_position != NULL) && (servo->mode == SERVO_MODE_POSITION_FEEDBACK)){
            // Обратная связь по положению
            targer_rpm = PIDController_Update(servo->pid_position, servo->target_postion, position);
            // Если превысили допустиму скорость, ограничиваем
            if(fabsf(targer_rpm) > fabsf(servo->target_rpm)){
                if((targer_rpm * servo->target_rpm) > 0){
                    targer_rpm =  servo->target_rpm;
                }else{
                    targer_rpm = -servo->target_rpm;
                }
            }
        }

        // Защита от осциляций на предельно малых скоростях
        if(fabsf(targer_rpm) < servo->min_rpm){
            targer_rpm = 0.0f;
            servo->pid_rpm->integrator = 0.0f;
        }

        // Обратная связь по скорости
        float power = PIDController_Update(servo->pid_rpm, targer_rpm, rpm);

        // Если превышение мощности, ограничиваем
        if(fabsf(power) > servo->max_power){
            if(power > 0){
                power =  servo->max_power;
            }else{
                power = -servo->max_power;
            }
        }
        servo->power = power;

        res = motord_duty_set(servo->motord, power);
    }
    
    return res;
}
