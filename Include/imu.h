#ifndef IMU_H__
#define IMU_H__

#include "driver_mpu9250_interface.h"

#define MPU9250_BASIC_DEFAULT_CLOCK_SOURCE                   MPU9250_CLOCK_SOURCE_PLL                  /**< pll */
#define MPU9250_BASIC_DEFAULT_RATE                           50                                        /**< 50Hz */
#define MPU9250_BASIC_DEFAULT_LOW_PASS_FILTER                MPU9250_LOW_PASS_FILTER_0                 /**< low pass filter 3 */
#define MPU9250_BASIC_DEFAULT_CYCLE_WAKE_UP                  MPU9250_BOOL_FALSE                        /**< disable cycle wake up */
#define MPU9250_BASIC_DEFAULT_INTERRUPT_PIN_LEVEL            MPU9250_PIN_LEVEL_LOW                     /**< low level */
#define MPU9250_BASIC_DEFAULT_INTERRUPT_PIN_TYPE             MPU9250_PIN_TYPE_PUSH_PULL                /**< push pull */
#define MPU9250_BASIC_DEFAULT_ACCELEROMETER_RANGE            MPU9250_ACCELEROMETER_RANGE_4G            /**< 2g */
#define MPU9250_BASIC_DEFAULT_GYROSCOPE_RANGE                MPU9250_GYROSCOPE_RANGE_2000DPS           /**< 2000dps */
#define MPU9250_BASIC_DEFAULT_INTERRUPT_MOTION               MPU9250_BOOL_FALSE                        /**< disable motion */
#define MPU9250_BASIC_DEFAULT_INTERRUPT_FIFO_OVERFLOW        MPU9250_BOOL_FALSE                        /**< disable fifo overflow */
#define MPU9250_BASIC_DEFAULT_INTERRUPT_DMP                  MPU9250_BOOL_FALSE                        /**< disable dmp */
#define MPU9250_BASIC_DEFAULT_INTERRUPT_FSYNC_INT            MPU9250_BOOL_FALSE                        /**< disable fsync int */
#define MPU9250_BASIC_DEFAULT_INTERRUPT_DATA_READY           MPU9250_BOOL_FALSE                        /**< disable data ready */
#define MPU9250_BASIC_DEFAULT_INTERRUPT_LATCH                MPU9250_BOOL_TRUE                         /**< enable latch */
#define MPU9250_BASIC_DEFAULT_INTERRUPT_READ_CLEAR           MPU9250_BOOL_TRUE                         /**< enable interrupt read clear */
#define MPU9250_BASIC_DEFAULT_EXTERN_SYNC                    MPU9250_EXTERN_SYNC_INPUT_DISABLED        /**< extern sync input disable */
#define MPU9250_BASIC_DEFAULT_FSYNC_INTERRUPT                MPU9250_BOOL_FALSE                        /**< disable fsync interrupt */
#define MPU9250_BASIC_DEFAULT_FSYNC_INTERRUPT_LEVEL          MPU9250_PIN_LEVEL_LOW                     /**< low level */
#define MPU9250_BASIC_DEFAULT_IIC_MASTER                     MPU9250_BOOL_FALSE                        /**< disable iic master */
#define MPU9250_BASIC_DEFAULT_IIC_BYPASS                     MPU9250_BOOL_FALSE                        /**< disable iic bypass */
#define MPU9250_BASIC_DEFAULT_GYROSCOPE_STANDBY              MPU9250_BOOL_FALSE                        /**< disable gyro standby */
#define MPU9250_BASIC_DEFAULT_FIFO_MODE                      MPU9250_FIFO_MODE_NORMAL                  /**< normal mode */
#define MPU9250_BASIC_DEFAULT_GYROSCOPE_CHOICE               0                                         /**< 0 */
#define MPU9250_BASIC_DEFAULT_ACCELEROMETER_CHOICE           0                                         /**< 0 */
#define MPU9250_BASIC_DEFAULT_ACCELEROMETER_LOW_PASS_FILTER  MPU9250_ACCELEROMETER_LOW_PASS_FILTER_3   /**< low pass filter 3 */
#define MPU9250_BASIC_DEFAULT_LOW_POWER_ACCEL_OUTPUT_RATE    MPU9250_LOW_POWER_ACCEL_OUTPUT_RATE_62P50 /**< 62.5Hz */
#define MPU9250_BASIC_DEFAULT_WAKE_ON_MOTION                 MPU9250_BOOL_FALSE                        /**< disable wake on motion */
#define MPU9250_BASIC_DEFAULT_ACCELEROMETER_COMPARE          MPU9250_BOOL_TRUE                         /**< enable compare */
#define MPU9250_BASIC_DEFAULT_MAGNETOMETER_MODE              MPU9250_MAGNETOMETER_MODE_POWER_DOWN     /**< 100Hz */
#define MPU9250_BASIC_DEFAULT_MAGNETOMETER_BITS              MPU9250_MAGNETOMETER_BITS_16              /**< 16bits */

#define MPU9250_SPI_INTERFACE     hspi3

int imu_init( void );
uint8_t imu_read_temperature(float *degrees);
uint8_t imu_read(float g[3], float dps[3]);

#endif /* IMU_H__ */
