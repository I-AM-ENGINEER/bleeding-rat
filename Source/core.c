#include "core.h"
#include "move.h"
#include "shell.h"
#include "system.h"

void core_init( void ){
	move_init(MOTOR_DECAY_FAST);
	move_permit(true);
	move_set_speed(0.0, 0.0);
	shell_log("[move] init ok");
}

void core_loop( void ){
	delay(1);
}
