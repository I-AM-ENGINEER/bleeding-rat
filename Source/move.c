#include <stdio.h>
#include <string.h>
#include "lwshell/lwshell.h"
#include "move.h"
#include "shell.h"
#include "main.h"

static move_t movement;

extern TIM_HandleTypeDef MOTOR_L_TIM;
extern TIM_HandleTypeDef MOTOR_L_TIM;

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
                motord_set_speed(&movement.motor_l, new_speed);
            }
        }else if(!strcmp(argument, "-sr")){
            if(next_arg != NULL){
                float new_speed;
                sscanf(next_arg, "%f", &new_speed);
                motord_set_speed(&movement.motor_r, new_speed);
            }
        }else if(!strcmp(argument, "-s")){
            if(next_arg != NULL){
                float new_speed;
                sscanf(next_arg, "%f", &new_speed);
                motord_set_speed(&movement.motor_l, new_speed);
                motord_set_speed(&movement.motor_r, new_speed);
            }
        }else if(!strcmp(argument, "-b")){
            motord_set_speed(&movement.motor_l, 0);
            motord_set_speed(&movement.motor_r, 0);
        }else{
            printf("unsupported cmd, type \"move -h\" for help\r\n");
            return -1;
        }
    }
    return 0;
}

int32_t move_init( void ){
    lwshell_register_cmd("move", move_cmd_move, "manual motors control");
    int32_t res = 0;
    int32_t tmp_res = 0;

    tmp_res = motord_init(&movement.motor_l, &MOTOR_L_TIM, MOTOR_L_PIN2_TIM_CHANNEL, MOTOR_L_PIN1_TIM_CHANNEL, MOTOR_EN_GPIO, MOTOR_EN_PIN);
    if(tmp_res != 0){
        res = 1;
        shell_log("[move] motor left init fault");
    }else{
        motord_set_decay(&movement.motor_l, MOTOR_DECAY_FAST);
    }

    tmp_res = motord_init(&movement.motor_r, &MOTOR_R_TIM, MOTOR_R_PIN2_TIM_CHANNEL, MOTOR_R_PIN1_TIM_CHANNEL, MOTOR_EN_GPIO, MOTOR_EN_PIN);
    if(tmp_res != 0){
        res = -1;
        shell_log("[move] motor right init fault");
    }else{
        motord_set_decay(&movement.motor_r, MOTOR_DECAY_FAST);
    }

    tmp_res = encoderd_init(&movement.encoder_lb,\
        ENCODER_L_B_PINA_PORT, ENCODER_L_B_PINA_PIN,\
        ENCODER_L_B_PINB_PORT, ENCODER_L_B_PINB_PIN\
    );
    if(tmp_res != 0){
        res = -1;
        shell_log("[move] encoder left back init fault");
    }

    tmp_res = encoderd_init(&movement.encoder_lf,\
        ENCODER_L_F_PINA_PORT, ENCODER_L_F_PINA_PIN,\
        ENCODER_L_F_PINB_PORT, ENCODER_L_F_PINB_PIN\
    );
    if(tmp_res != 0){
        res = -1;
        shell_log("[move] encoder left front init fault");
    }

    tmp_res = encoderd_init(&movement.encoder_rb,\
        ENCODER_R_B_PINA_PORT, ENCODER_R_B_PINA_PIN,\
        ENCODER_R_B_PINB_PORT, ENCODER_R_B_PINB_PIN\
    );
    if(tmp_res != 0){
        res = -1;
        shell_log("[move] encoder right back init fault");
    }

    tmp_res = encoderd_init(&movement.encoder_rf,\
        ENCODER_R_F_PINA_PORT, ENCODER_R_F_PINA_PIN,\
        ENCODER_R_F_PINB_PORT, ENCODER_R_F_PINB_PIN\
    );
    if(tmp_res != 0){
        res = -1;
        shell_log("[move] encoder right front init fault");
    }
    return res;
}

void move_permit( bool permission ){
    #ifdef MOVE_DEBUG
    if(permission){
        shell_log("[move] motors enabled");
    }else{
        shell_log("[move] motors disabled");
    }
    #endif
    motord_enable(&movement.motor_l, permission);
    motord_enable(&movement.motor_r, permission);
}

void move_set_speed( float speed_l, float speed_r ){
    #ifdef MOVE_DEBUG
    shell_log("[move] motors speed set: l: %.3f, r: %.3f", \
        speed_l,\
        speed_r\
    );
    #endif
    motord_set_speed(&movement.motor_l, speed_l);
    motord_set_speed(&movement.motor_r, speed_r);
}

void move_encoders_process( void ){
    int32_t res = 0;
    res = encoderd_process(&movement.encoder_lb);
    if(res == -2){
        shell_log("[move] encoder left back skip step");
    }

    res = encoderd_process(&movement.encoder_lf);
    if(res == -2){
        shell_log("[move] encoder left front skip step");
    }

    res = encoderd_process(&movement.encoder_rb);
    if(res == -2){
        shell_log("[move] encoder right back skip step");
    }

    res = encoderd_process(&movement.encoder_rf);
    if(res == -2){
        shell_log("[move] encoder right front skip step");
    }
}

int32_t move_encoders_get( encoder_position_t encoder_position ){
    encoderd_t *encoderd;

    switch (encoder_position){
    case ENCODER_BACK_LEFT:
        encoderd = &movement.encoder_lb;
        break;
    case ENCODER_BACK_RIGHT:
        encoderd = &movement.encoder_rb;
        break;
    case ENCODER_FRONT_LEFT:
        encoderd = &movement.encoder_lf;
        break;
    case ENCODER_FRONT_RIGHT:
        encoderd = &movement.encoder_rf;
        break;
    default:
        break;
    }
    
    if(encoderd == NULL){
        return 0;
    }
    return encoderd_get_steps(encoderd);
}
