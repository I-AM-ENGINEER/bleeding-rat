#include "core.h"
#include "move.h"
#include "shell.h"
#include "system.h"
#include "imu.h"

void core_init( void ){
	move_init(MOTOR_DECAY_FAST);
	move_permit(true);
	move_set_speed(0.0, 0.0);
	shell_log("[move] init ok");
}

void core_loop( void ){
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
	delay(100);
}
