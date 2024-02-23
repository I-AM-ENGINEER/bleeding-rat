#include "core.h"
#include "ws2812b.h"

// Функция, которая будет вызываться при изменении состояния любого сенсора
void collision_event( uint16_t sensor_num, collision_sensor_state_t event_type ){
	if(event_type == COLLISION_STATE_ENGAGE){
		shell_log("sensor %hu engage", sensor_num);
	}else{
		shell_log("sensor %hu disengage", sensor_num);
	}
}

void core_init( void ){
	ws2812b_write(0,0,0);
	move_servos_permit(false);
	shell_log("[move] init ok");
	collision_attach(collision_event);
	for(uint16_t i = 0; i < 5; i++){
		collision_sensor_set_comparator(i, 150, 100);
	}
}

void core_loop( void ){
	ws2812b_write(255,0,0);
	delay(300);
	ws2812b_write(0,255,0);
	delay(300);
	ws2812b_write(0,0,255);
	delay(300);

	//move_servos_position_set(200.0f, 200.0f, 240.0f);
	//delay(3000);
	//move_servos_position_set(-200.0f, -200.0f, 240.0f);
	//delay(3000);
}


