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

int32_t move_init( enum motor_decay_mode_e decay_mode ){
    lwshell_register_cmd("move", move_cmd_move, "manual motors control");
    motord_init(&movement.motor_l, &MOTOR_L_TIM, MOTOR_L_PIN1_TIM_CHANNEL, MOTOR_L_PIN2_TIM_CHANNEL, MOTOR_EN_GPIO, MOTOR_EN_PIN);
	motord_init(&movement.motor_r, &MOTOR_R_TIM, MOTOR_R_PIN1_TIM_CHANNEL, MOTOR_R_PIN2_TIM_CHANNEL, MOTOR_EN_GPIO, MOTOR_EN_PIN);
    motord_set_decay(&movement.motor_l, decay_mode);
    motord_set_decay(&movement.motor_r, decay_mode);
    return 0;
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
        speed_l,
        speed_r
    );
    #endif
    motord_set_speed(&movement.motor_l, speed_l);
    motord_set_speed(&movement.motor_r, speed_r);
}

void move_motors_mode(  ){

}
