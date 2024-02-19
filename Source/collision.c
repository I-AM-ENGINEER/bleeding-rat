#include "collision.h"
#include <stdlib.h>

static collision_sensor_t sensors[COLLISION_SENSORS_COUNT];
static callback_t collision_callback;

void collision_attach( callback_t callback ){
    collision_callback = callback;
}

void collision_process( uint16_t *new_sensors_data ){
    for(uint16_t i = 0; i < COLLISION_SENSORS_COUNT; i++){
        sensors[i].last_value = sensors[i].current_value;
        sensors[i].current_value = new_sensors_data[i];
        if((sensors[i].current_value >  sensors[i].comp_engage_threshold) && \
              (sensors[i].comp_state == COLLISION_EVENT_DISENGAGE))
        {   
            sensors[i].comp_state = COLLISION_EVENT_ENGAGE;
            if(collision_callback != NULL){
                collision_callback(i, COLLISION_EVENT_ENGAGE);
            }
        }
        if((sensors[i].current_value <  sensors[i].comp_disengage_threshold) && \
              (sensors[i].comp_state == COLLISION_EVENT_ENGAGE))
        {
            sensors[i].comp_state = COLLISION_EVENT_DISENGAGE;
            if(collision_callback != NULL){
                collision_callback(i, COLLISION_EVENT_DISENGAGE);
            }
        }
    }
}

uint16_t collision_sensor_get_value( uint16_t sensor_num ){
    if(sensor_num >= COLLISION_SENSORS_COUNT){
        return 0;
    }
    return sensors[sensor_num].current_value;
}

collision_sensor_state_t collision_sensor_get_state( uint16_t sensor_num ){
    if(sensor_num >= COLLISION_SENSORS_COUNT){
        return 0;
    }
    return sensors[sensor_num].comp_state;
}

void collision_sensor_set_comparator( uint16_t sensor_num, uint16_t engage_threshold, uint16_t disengage_threshold ){
    if(sensor_num >= COLLISION_SENSORS_COUNT){
        return;
    }
    if(engage_threshold < disengage_threshold){
        engage_threshold = disengage_threshold;
    }
    sensors[sensor_num].comp_engage_threshold = engage_threshold;
    sensors[sensor_num].comp_disengage_threshold = disengage_threshold;
}
