#include "core.h"
#include "move.h"
#include "shell.h"
#include "system.h"
#include "imu.h"
#include "collision.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim8;

void print_imu( void ){
	float a[3];
	float b[3];
	float t;
	uint8_t res = 0;

	res = imu_read(a, b);
	if(res != 0){
		shell_log("[imu] read error");
	}

	res = imu_read_temperature(&t);
	if(res != 0){
		shell_log("[imu] read error");
	}
	shell_log("temp:%.1f\t\tacc:%.3f\t%.3f\t%.3f\t\tgyro:%.3f\t%.3f\t%.3f\t", t, a[0], a[1], a[2], b[0], b[1], b[2]);
}

void collision_event( int16_t sensor_num, collision_sensor_state_t event_type ){
	if(event_type == COLLISION_EVENT_ENGAGE){
		shell_log("[collision] sensor %hu engage", sensor_num);
	}else{
		shell_log("[collision] sensor %hu disengage", sensor_num);
	}
}

void core_init( void ){
	move_init(MOTOR_DECAY_FAST);
	move_permit(true);
	move_set_speed(0.0, 0.0);
	shell_log("[move] init ok");
	collision_attach(collision_event);
	for(uint16_t i = 0; i < 5; i++){
		collision_sensor_set_comparator(i, 150, 100);
	}
}

void core_loop( void ){
	while(collision_sensor_get_state(4) == COLLISION_EVENT_DISENGAGE){
		move_set_speed(0.4, 0.4);
	}
	move_set_speed(-0.4, -0.4);
	delay(500);
	move_set_speed(0.4, -0.4);
	delay(500);


	/*
	for(uint32_t i = 0; i < 5; i++){
		printf("%d\t", collision_sensor_get_value(i));
	}
	printf("\r\n");
	delay(1000);
	*/
}
