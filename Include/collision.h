#ifndef COLLISION_H__
#define COLLISION_H__

#define COLLISION_DEFAULT_ENGAGE_THRESHOLD          300
#define COLLISION_DEFAULT_DISENGAGE_THRESHOLD       500


#include <stdint.h>
#include <stdbool.h>

typedef enum {
    COLLISION_STATE_NONE,
    COLLISION_STATE_ENGAGE,
    COLLISION_STATE_DISENGAGE,
} collision_sensor_state_t;

typedef struct{
    uint16_t current_value;
    uint16_t last_value;
    uint16_t comp_engage_threshold;
    uint16_t comp_disengage_threshold;
    collision_sensor_state_t comp_state;
} collision_sensor_t;

typedef void (*callback_t)( uint16_t sensor_num, collision_sensor_state_t event_type );

int32_t collision_init( collision_sensor_t *sensors_array, uint16_t sensors_cnt );
int32_t collision_process( uint16_t new_sensor_data, uint16_t sensor_num );
int32_t collision_attach( callback_t callback );
int32_t collision_sensor_set_comparator( uint16_t sensor_num, uint16_t engage_threshold, uint16_t disengage_threshold );
uint16_t collision_sensor_get_value( uint16_t sensor_num );
collision_sensor_state_t collision_sensor_get_state( uint16_t sensor_num );

#endif // COLLISION_H__