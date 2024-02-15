#include "imu.h"
#include "stm32f4xx.h"

static mpu9250_handle_t imu_handle; 

/**
 * @brief     basic example init
 * @param[in] interface is the used interface
 * @param[in] addr_pin is the iic device address
 * @return    status code
 *            - 0 success
 *            - 1 init failed
 * @note      spi can't read magnetometer data
 */
int imu_init( void ){
    //mpu9250_interface_t interface, mpu9250_address_t addr_pin

    mpu9250_interface_t interface = MPU9250_INTERFACE_SPI;

    uint8_t res;
    
    /* link interface function */
    DRIVER_MPU9250_LINK_INIT(&imu_handle, mpu9250_handle_t);
    DRIVER_MPU9250_LINK_IIC_INIT(&imu_handle, mpu9250_interface_iic_init);
    DRIVER_MPU9250_LINK_IIC_DEINIT(&imu_handle, mpu9250_interface_iic_deinit);
    DRIVER_MPU9250_LINK_IIC_READ(&imu_handle, mpu9250_interface_iic_read);
    DRIVER_MPU9250_LINK_IIC_WRITE(&imu_handle, mpu9250_interface_iic_write);
    DRIVER_MPU9250_LINK_SPI_INIT(&imu_handle, mpu9250_interface_spi_init);
    DRIVER_MPU9250_LINK_SPI_DEINIT(&imu_handle, mpu9250_interface_spi_deinit);
    DRIVER_MPU9250_LINK_SPI_READ(&imu_handle, mpu9250_interface_spi_read);
    DRIVER_MPU9250_LINK_SPI_WRITE(&imu_handle, mpu9250_interface_spi_write);
    DRIVER_MPU9250_LINK_DELAY_MS(&imu_handle, mpu9250_interface_delay_ms);
    DRIVER_MPU9250_LINK_DEBUG_PRINT(&imu_handle, mpu9250_interface_debug_print);
    DRIVER_MPU9250_LINK_RECEIVE_CALLBACK(&imu_handle, mpu9250_interface_receive_callback);
    
    /* set the interface */
    res = mpu9250_set_interface(&imu_handle, interface);
    if (res != 0){
        mpu9250_interface_debug_print("mpu9250: set interface failed.\n");
        return 1;
    }

    
    mpu9250_interface_delay_ms(50);
    uint32_t time = HAL_GetTick();
    while((HAL_GetTick() - time) < 300){
        res = mpu9250_init(&imu_handle);
        if (res == 0){
            break;
        }
        mpu9250_interface_delay_ms(50);
    }

    if(res != 0){
        mpu9250_interface_debug_print("mpu9250: init failed (timeout).\n");
        return 1;
    }
    
    /* delay 100 ms */
    mpu9250_interface_delay_ms(100);
    
    /* disable sleep */
    res = mpu9250_set_sleep(&imu_handle, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set sleep failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* if spi interface, disable iic interface */
    if (interface == MPU9250_INTERFACE_SPI)
    {
        /* disable iic */
        res = mpu9250_set_disable_iic_slave(&imu_handle, MPU9250_BOOL_TRUE);
        if (res != 0)
        {
            mpu9250_interface_debug_print("mpu9250: set disable iic slave failed.\n");
            (void)mpu9250_deinit(&imu_handle);
           
            return 1;
        }
    }
    
    /* set fifo 1024kb */
    res = mpu9250_set_fifo_1024kb(&imu_handle);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set fifo 1024kb failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default clock source */
    res = mpu9250_set_clock_source(&imu_handle, MPU9250_BASIC_DEFAULT_CLOCK_SOURCE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set clock source failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default rate */
    res = mpu9250_set_sample_rate_divider(&imu_handle, 1000 / (MPU9250_BASIC_DEFAULT_RATE - 1));
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set sample rate divider failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* enable temperature sensor */
    res = mpu9250_set_ptat(&imu_handle, MPU9250_BOOL_TRUE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set ptat failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default cycle wake up */
    res = mpu9250_set_cycle_wake_up(&imu_handle, MPU9250_BASIC_DEFAULT_CYCLE_WAKE_UP);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set cycle wake up failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* enable acc x */
    res = mpu9250_set_standby_mode(&imu_handle, MPU9250_SOURCE_ACC_X, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set standby mode failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* enable acc y */
    res = mpu9250_set_standby_mode(&imu_handle, MPU9250_SOURCE_ACC_Y, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set standby mode failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* enable acc z */
    res = mpu9250_set_standby_mode(&imu_handle, MPU9250_SOURCE_ACC_Z, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set standby mode failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* enable gyro x */
    res = mpu9250_set_standby_mode(&imu_handle, MPU9250_SOURCE_GYRO_X, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set standby mode failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* enable gyro y */
    res = mpu9250_set_standby_mode(&imu_handle, MPU9250_SOURCE_GYRO_Y, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set standby mode failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* enable gyro z */
    res = mpu9250_set_standby_mode(&imu_handle, MPU9250_SOURCE_GYRO_Z, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set standby mode failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* disable gyroscope x test */
    res = mpu9250_set_gyroscope_test(&imu_handle, MPU9250_AXIS_X, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set gyroscope test failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* disable gyroscope y test */
    res = mpu9250_set_gyroscope_test(&imu_handle, MPU9250_AXIS_Y, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set gyroscope test failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* disable gyroscope z test */
    res = mpu9250_set_gyroscope_test(&imu_handle, MPU9250_AXIS_Z, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set gyroscope test failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* disable accelerometer x test */
    res = mpu9250_set_accelerometer_test(&imu_handle, MPU9250_AXIS_X, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set accelerometer test failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* disable accelerometer y test */
    res = mpu9250_set_accelerometer_test(&imu_handle, MPU9250_AXIS_Y, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set accelerometer test failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* disable accelerometer z test */
    res = mpu9250_set_accelerometer_test(&imu_handle, MPU9250_AXIS_Z, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set accelerometer test failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* disable fifo */
    res = mpu9250_set_fifo(&imu_handle, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set fifo failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* disable temp fifo */
    res = mpu9250_set_fifo_enable(&imu_handle, MPU9250_FIFO_TEMP, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set fifo enable failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* disable xg fifo */
    res = mpu9250_set_fifo_enable(&imu_handle, MPU9250_FIFO_XG, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set fifo enable failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* disable yg fifo */
    res = mpu9250_set_fifo_enable(&imu_handle, MPU9250_FIFO_YG, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set fifo enable failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* disable zg fifo */
    res = mpu9250_set_fifo_enable(&imu_handle, MPU9250_FIFO_ZG, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set fifo enable failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* disable accel fifo */
    res = mpu9250_set_fifo_enable(&imu_handle, MPU9250_FIFO_ACCEL, MPU9250_BOOL_FALSE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set fifo enable failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default interrupt level */
    res = mpu9250_set_interrupt_level(&imu_handle, MPU9250_BASIC_DEFAULT_INTERRUPT_PIN_LEVEL);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set interrupt level failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default pin type */
    res = mpu9250_set_interrupt_pin_type(&imu_handle, MPU9250_BASIC_DEFAULT_INTERRUPT_PIN_TYPE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set interrupt pin type failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default motion interrupt */
    res = mpu9250_set_interrupt(&imu_handle, MPU9250_INTERRUPT_MOTION, MPU9250_BASIC_DEFAULT_INTERRUPT_MOTION);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set interrupt failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default fifo overflow interrupt */
    res = mpu9250_set_interrupt(&imu_handle, MPU9250_INTERRUPT_FIFO_OVERFLOW, MPU9250_BASIC_DEFAULT_INTERRUPT_FIFO_OVERFLOW);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set interrupt failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default dmp interrupt */
    res = mpu9250_set_interrupt(&imu_handle, MPU9250_INTERRUPT_DMP, MPU9250_BASIC_DEFAULT_INTERRUPT_DMP);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set interrupt failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default fsync int interrupt */
    res = mpu9250_set_interrupt(&imu_handle, MPU9250_INTERRUPT_FSYNC_INT, MPU9250_BASIC_DEFAULT_INTERRUPT_FSYNC_INT);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set interrupt failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default data ready interrupt */
    res = mpu9250_set_interrupt(&imu_handle, MPU9250_INTERRUPT_DATA_READY, MPU9250_BASIC_DEFAULT_INTERRUPT_DATA_READY);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set interrupt failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default interrupt latch */
    res = mpu9250_set_interrupt_latch(&imu_handle, MPU9250_BASIC_DEFAULT_INTERRUPT_LATCH);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set interrupt latch failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default interrupt read clear */
    res = mpu9250_set_interrupt_read_clear(&imu_handle, MPU9250_BASIC_DEFAULT_INTERRUPT_READ_CLEAR);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set interrupt read clear failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the extern sync */
    res = mpu9250_set_extern_sync(&imu_handle, MPU9250_BASIC_DEFAULT_EXTERN_SYNC);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set extern sync failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default fsync interrupt */
    res = mpu9250_set_fsync_interrupt(&imu_handle, MPU9250_BASIC_DEFAULT_FSYNC_INTERRUPT);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set fsync interrupt failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default fsync interrupt level */
    res = mpu9250_set_fsync_interrupt_level(&imu_handle, MPU9250_BASIC_DEFAULT_FSYNC_INTERRUPT_LEVEL);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set fsync interrupt level failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default iic master */
    res = mpu9250_set_iic_master(&imu_handle, MPU9250_BASIC_DEFAULT_IIC_MASTER);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set iic master failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default iic bypass */
    res = mpu9250_set_iic_bypass(&imu_handle, MPU9250_BASIC_DEFAULT_IIC_BYPASS);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set iic bypass failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default accelerometer range */
    res = mpu9250_set_accelerometer_range(&imu_handle, MPU9250_BASIC_DEFAULT_ACCELEROMETER_RANGE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set accelerometer range failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default gyroscope range */
    res = mpu9250_set_gyroscope_range(&imu_handle, MPU9250_BASIC_DEFAULT_GYROSCOPE_RANGE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set gyroscope range failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default gyro standby */
    res = mpu9250_set_gyro_standby(&imu_handle, MPU9250_BASIC_DEFAULT_GYROSCOPE_STANDBY);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set gyro standby failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default fifo mode */
    res = mpu9250_set_fifo_mode(&imu_handle, MPU9250_BASIC_DEFAULT_FIFO_MODE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set fifo mode failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default gyroscope choice */
    res = mpu9250_set_gyroscope_choice(&imu_handle, MPU9250_BASIC_DEFAULT_GYROSCOPE_CHOICE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set gyroscope choice failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default low pass filter */
    res = mpu9250_set_low_pass_filter(&imu_handle, MPU9250_BASIC_DEFAULT_LOW_PASS_FILTER);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set low pass filter failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default accelerometer choice */
    res = mpu9250_set_accelerometer_choice(&imu_handle, MPU9250_BASIC_DEFAULT_ACCELEROMETER_CHOICE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set accelerometer choice failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default accelerometer low pass filter */
    res = mpu9250_set_accelerometer_low_pass_filter(&imu_handle, MPU9250_BASIC_DEFAULT_ACCELEROMETER_LOW_PASS_FILTER);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set accelerometer low pass filter failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default low power accel output rate */
    res = mpu9250_set_low_power_accel_output_rate(&imu_handle, MPU9250_BASIC_DEFAULT_LOW_POWER_ACCEL_OUTPUT_RATE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set low power accel output rate failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default wake on motion */
    res = mpu9250_set_wake_on_motion(&imu_handle, MPU9250_BASIC_DEFAULT_WAKE_ON_MOTION);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set wake on motion failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* set the default accel compare with previous sample */
    res = mpu9250_set_accel_compare_with_previous_sample(&imu_handle, MPU9250_BASIC_DEFAULT_ACCELEROMETER_COMPARE);
    if (res != 0)
    {
        mpu9250_interface_debug_print("mpu9250: set accel compare with previous sample failed.\n");
        (void)mpu9250_deinit(&imu_handle);
       
        return 1;
    }
    
    /* if iic interface */
    if (interface == MPU9250_INTERFACE_IIC)
    {
        /* mag init */
        res = mpu9250_mag_init(&imu_handle); 
        if (res != 0)
        {
            mpu9250_interface_debug_print("mpu9250: mag init failed.\n");
            (void)mpu9250_deinit(&imu_handle);
           
            return 1;
        }
        
        /* set the mag default mode */
        res = mpu9250_mag_set_mode(&imu_handle, MPU9250_BASIC_DEFAULT_MAGNETOMETER_MODE);
        if (res != 0)
        {
            mpu9250_interface_debug_print("mpu9250: mag set mode failed.\n");
            (void)mpu9250_mag_deinit(&imu_handle); 
            (void)mpu9250_deinit(&imu_handle);
           
            return 1;
        }
        
        /* set the mag default bits */
        res = mpu9250_mag_set_bits(&imu_handle, MPU9250_BASIC_DEFAULT_MAGNETOMETER_BITS);
        if (res != 0)
        {
            mpu9250_interface_debug_print("mpu9250: mag set bits failed.\n");
            (void)mpu9250_mag_deinit(&imu_handle); 
            (void)mpu9250_deinit(&imu_handle);
           
            return 1;
        }
    }
    
    return 0;
}

/**
 * @brief      basic example read temperature
 * @param[out] *degrees points to a converted data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read temperature failed
 * @note       none
 */
uint8_t imu_read_temperature(float *degrees)
{
    int16_t raw;
    
    /* read temperature */
    if (mpu9250_read_temperature(&imu_handle, &raw, degrees) != 0)
    {
        return 1;
    }
    
    return 0;
}

/**
 * @brief      basic example read
 * @param[out] *g points to a converted data buffer
 * @param[out] *dps points to a converted data buffer
 * @param[out] *ut points to a converted data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t imu_read(float g[3], float dps[3])
{
    uint16_t len;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t mag_raw[3];
    float accel[3];
    float gyro[3];
    float mag[3];
    
    /* set 1 */
    len = 1;
    
    /* read data */
    if (mpu9250_read(&imu_handle,
                    (int16_t (*)[3])&accel_raw, (float (*)[3])&accel,
                    (int16_t (*)[3])&gyro_raw, (float (*)[3])&gyro,
                    (int16_t (*)[3])&mag_raw, (float (*)[3])&mag,
                     &len) != 0
                    )
    {
        return 1;
    }
    
    /* copy the data */
    g[0] = accel[0];
    g[1] = accel[1];
    g[2] = accel[2];
    dps[0] = gyro[0];
    dps[1] = gyro[1];
    dps[2] = gyro[2];
    
    return 0;
}
