#include "core.h"
#include "move.h"
#include "shell.h"
#include "stm32f4xx_hal.h"
#include "system.h"

void core_init( void ){
	sys_init();
	move_permit(true);
}

void core_loop( void ){
	delay_ms(1);
}
