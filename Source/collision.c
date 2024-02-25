#include "collision.h"
#include <stdlib.h>

static uint16_t sensors_count;
static collision_sensor_t *sensors;
static callback_t collision_callback;

int32_t collision_init( collision_sensor_t *sensors_array, uint16_t sensors_cnt ){
    if(sensors_array == NULL){
        return -1;
    }

    sensors_count = sensors_cnt;
    sensors = sensors_array;
    for(uint32_t i = 0; i < sensors_count; i++){
        sensors[i].threshold_disengage = COLLISION_DEFAULT_DISENGAGE_THRESHOLD;
        sensors[i].threshold_disengage = COLLISION_DEFAULT_ENGAGE_THRESHOLD;
        sensors[i].state = COLLISION_STATE_DISENGAGE;
        sensors[i].value_last = 0;
        sensors[i].value_current = 0;
    }
    return 0;
}

int32_t collision_attach( callback_t callback ){
    if(callback == NULL){
        return -1;
    }

    collision_callback = callback;
    
    return 0;
}

int32_t collision_process( uint16_t sensor_num, uint16_t new_sensor_data ){
    if(sensor_num >= sensors_count){
        return -1;
    }

    sensors[sensor_num].value_last = sensors[sensor_num].value_current;
    sensors[sensor_num].value_current = new_sensor_data;
    if((sensors[sensor_num].value_current >  sensors[sensor_num].threshold_engage) && \
          (sensors[sensor_num].state == COLLISION_STATE_DISENGAGE))
    {   
        sensors[sensor_num].state = COLLISION_STATE_ENGAGE;
        if(collision_callback != NULL){
            collision_callback(sensor_num, COLLISION_STATE_ENGAGE);
        }
    }
    if((sensors[sensor_num].value_current <  sensors[sensor_num].threshold_disengage) && \
          (sensors[sensor_num].state == COLLISION_STATE_ENGAGE))
    {
        sensors[sensor_num].state = COLLISION_STATE_DISENGAGE;
        if(collision_callback != NULL){
            collision_callback(sensor_num, COLLISION_STATE_DISENGAGE);
        }
    }

    return 0;
}

uint16_t collision_get_value( uint16_t sensor_num ){
    if(sensor_num >= sensors_count){
        return 0;
    }

    return sensors[sensor_num].value_current;
}

collision_state_t collision_get_state( uint16_t sensor_num ){
    if(sensor_num >= sensors_count){
        return COLLISION_STATE_NONE;
    }
    
    return sensors[sensor_num].state;
}

int32_t collision_set_comparator( uint16_t sensor_num, uint16_t engage_threshold, uint16_t disengage_threshold ){
    if(sensor_num >= sensors_count){
        return -1;
    }

    if(engage_threshold < disengage_threshold){
        engage_threshold = disengage_threshold;
    }
    sensors[sensor_num].threshold_engage = engage_threshold;
    sensors[sensor_num].threshold_disengage = disengage_threshold;

    return 0;
}
