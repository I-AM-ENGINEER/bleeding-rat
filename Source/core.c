#include "core.h"

// Функция, которая будет вызываться при изменении состояния любого сенсора
void collision_event( uint16_t sensor_num, collision_sensor_state_t event_type ){
	if(event_type == COLLISION_STATE_ENGAGE){
		shell_log("sensor %hu engage", sensor_num);
	}else{
		shell_log("sensor %hu disengage", sensor_num);
	}
}

void core_init( void ){
	move_servos_permit(true);
	shell_log("[move] init ok");
	collision_attach(collision_event);
	for(uint16_t i = 0; i < 5; i++){
		collision_sensor_set_comparator(i, 150, 100);
	}
}



void core_loop( void ){
	move_servos_position_set(200.0f, 200.0f, 240.0f);
	delay(3000);
	move_servos_position_set(-200.0f, -200.0f, 240.0f);
	delay(3000);
}


