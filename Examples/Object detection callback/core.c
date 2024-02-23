#include "core.h"

// Значение при котором сенсор будет считать, что перед ним есть препядствие (рекомендую 50...1000), макссимально 4095
const uint16_t engage_threshold = 150;
// Значение при котором сенсор будет считать, что перед ним нет препядствия (рекомендую 50...1000), макссимально 4095
// Во избежания дребезна ДОЛЖНО быть меньше, чем engage_threshold
const uint16_t disengage_threshold = 100;

// Функция, которая будет вызываться при изменении состояния любого сенсора
void collision_event( uint16_t sensor_num, collision_sensor_state_t event_type ){
	if(event_type == COLLISION_STATE_ENGAGE){
		shell_log("sensor %hu engage", sensor_num);
	}else{
		shell_log("sensor %hu disengage", sensor_num);
	}
}

void core_init( void ){
    shell_log("start object detection callback example");

	// Настройка функции, которая будет вызываться при изменении состояния любого сенсора
	collision_attach(collision_event);
    
	// Настройка уровней срабатывания компараторов всех сенсоров препядствий
	collision_sensor_set_comparator(0, engage_threshold, disengage_threshold);
	collision_sensor_set_comparator(1, engage_threshold, disengage_threshold);
	collision_sensor_set_comparator(2, engage_threshold, disengage_threshold);
	collision_sensor_set_comparator(3, engage_threshold, disengage_threshold);
	collision_sensor_set_comparator(4, engage_threshold, disengage_threshold);
}

void core_loop( void ){
	// Имитация "занятостии" главного цикла
	delay(300);
	printf("some loop operation\r\n");
}
