#ifndef COLLISION_H__
#define COLLISION_H__

#define COLLISION_SENSORS_COUNT         5

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    COLLISION_EVENT_ENGAGE,
    COLLISION_EVENT_DISENGAGE,
} collision_sensor_state_t;

typedef struct{
    uint16_t current_value;
    uint16_t last_value;
    uint16_t comp_engage_threshold;
    uint16_t comp_disengage_threshold;
    collision_sensor_state_t comp_state;
} collision_sensor_t;

typedef void (*callback_t)( int16_t sensor_num, collision_sensor_state_t event_type );

void collision_process( uint16_t *new_sensors_data );
void collision_attach( callback_t callback );
void collision_sensor_set_comparator( uint16_t sensor_num, uint16_t engage_threshold, uint16_t disengage_threshold );
uint16_t collision_sensor_get_value( uint16_t sensor_num );
collision_sensor_state_t collision_sensor_get_state( uint16_t sensor_num );

#endif // COLLISION_H__