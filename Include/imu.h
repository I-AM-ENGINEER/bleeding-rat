#ifndef IMU_H__
#define IMU_H__

#include <stdint.h>

int32_t imu_init( void );
int32_t imu_read( float g[3], float dps[3] );
int32_t imu_read_temperature( float *temperature );

#endif /* IMU_H__ */
