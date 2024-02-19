#include "core.h"

// Значение при котором сенсор будет считать, что перед ним есть препядствие (рекомендую 50...1000), макссимально 4095
const uint16_t engage_threshold = 150;
// Значение при котором сенсор будет считать, что перед ним нет препядствия (рекомендую 50...1000), макссимально 4095
// Во избежания дребезна ДОЛЖНО быть меньше, чем engage_threshold
const uint16_t disengage_threshold = 100;
// Номер сенсора
const uint16_t colision_sensor_num = 4;

void core_init( void ){
    shell_log("start object detection example");
    // Настройка уровней срабатывания компаратора сенсоров препядствия
	collision_sensor_set_comparator(colision_sensor_num, engage_threshold, disengage_threshold);
}

void core_loop( void ){
    // Ждем, пока перед сенсором не появится препядствие
	while(collision_sensor_get_state(colision_sensor_num) == COLLISION_EVENT_DISENGAGE){ };
    // Вывод сообщения об обнаружении барьера
    shell_log("barrier engage");
    // Ждем, пока перед сенсором исчезнет препядствие
	while(collision_sensor_get_state(colision_sensor_num) == COLLISION_EVENT_ENGAGE){ };    
    // Вывод сообщения об исчезновении барьера
    shell_log("barrier disengage");
}
