/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_mpu9250_interface_template.c
 * @brief     driver mpu9250 interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-08-30
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/08/30  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include <stdarg.h>
#include "imu.h"
#include "stm32f4xx.h"
#include "driver_mpu9250_interface.h"
#include "shell.h"
#include "cmsis_os.h"
#include "main.h"

extern SPI_HandleTypeDef MPU9250_SPI_INTERFACE;

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t mpu9250_interface_iic_init(void)
{
    return 1;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t mpu9250_interface_iic_deinit(void)
{
    return 1;
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr is the iic device write address
 * @param[in]  reg is the iic register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t mpu9250_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return 1;
}

/**
 * @brief     interface iic bus write
 * @param[in] addr is the iic device write address
 * @param[in] reg is the iic register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t mpu9250_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return 1;
}

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t mpu9250_interface_spi_init(void)
{
    return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t mpu9250_interface_spi_deinit(void)
{   
    return 0;
}

static osThreadId_t thread_id;
enum send_state_e {
    SEND_NONE,
    SEND_ADDR,
    SEND_TX_DATA,
    SEND_RX_DATA,
};

static uint8_t* rx_buffer;
static uint8_t* tx_buffer;
static size_t rx_len;
static size_t tx_len;

volatile enum send_state_e send_state = SEND_NONE;

void HAL_SPI_RxCpltCallback( SPI_HandleTypeDef *hspi ){
    if(send_state == SEND_RX_DATA){
        osThreadFlagsSet(thread_id, 2);
    }
}

void HAL_SPI_TxCpltCallback( SPI_HandleTypeDef *hspi ){
    if(send_state == SEND_ADDR){
        osThreadFlagsSet(thread_id, 1);
    }else if(send_state == SEND_TX_DATA){
        osThreadFlagsSet(thread_id, 2);
    }
}

/**
 * @brief      interface spi bus read
 * @param[in]  reg is the register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t mpu9250_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef hal_res;
    int32_t os_res;
    uint8_t res = 0;
    uint8_t addr = reg | 0x80;

    if(send_state != SEND_NONE){
        return 1;
    }
    
    MP6500_CS_GPIO_Port->BSRR = (uint32_t)MP6500_CS_Pin << 16U;

    thread_id = osThreadGetId();
    if(thread_id == NULL){
        res = 1;
        goto end;
    }

    send_state = SEND_ADDR;
    osThreadFlagsClear(1);
    hal_res = HAL_SPI_Transmit_DMA(&MPU9250_SPI_INTERFACE, &addr, 1);
    if(hal_res != HAL_OK){
        res = 1;
        goto end;
    }
    
    os_res = (int32_t)osThreadFlagsWait(1, 0, 10);
    if(os_res < 0){
        res = 1;
        goto end;
    }
    
    send_state = SEND_RX_DATA;
    osThreadFlagsClear(2);
    hal_res = HAL_SPI_Receive_DMA(&MPU9250_SPI_INTERFACE, buf, len);
    if(hal_res != HAL_OK){
        res = 1;
        goto end;
    }

    os_res = (int32_t)osThreadFlagsWait(2, 0, 10);
    if(os_res < 0){
        res = 1;
        goto end;
    }
end:
    MP6500_CS_GPIO_Port->BSRR = MP6500_CS_Pin;
    send_state = SEND_NONE;
    return res;
}

/**
 * @brief     interface spi bus write
 * @param[in] reg is the register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t mpu9250_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef hal_res;
    int32_t os_res;
    uint8_t res = 0;
    uint8_t addr = reg | 0x80;

    if(send_state != SEND_NONE){
        return 1;
    }
    
    MP6500_CS_GPIO_Port->BSRR = (uint32_t)MP6500_CS_Pin << 16U;

    thread_id = osThreadGetId();
    if(thread_id == NULL){
        res = 1;
        goto end;
    }

    send_state = SEND_ADDR;
    osThreadFlagsClear(1);
    hal_res = HAL_SPI_Transmit_DMA(&MPU9250_SPI_INTERFACE, &addr, 1);
    if(hal_res != HAL_OK){
        res = 1;
        goto end;
    }
    
    os_res = (int32_t)osThreadFlagsWait(1, 0, 10);
    if(os_res < 0){
        res = 1;
        goto end;
    }
    
    send_state = SEND_TX_DATA;
    osThreadFlagsClear(2);
    hal_res = HAL_SPI_Transmit_DMA(&MPU9250_SPI_INTERFACE, buf, len);
    if(hal_res != HAL_OK){
        res = 1;
        goto end;
    }

    os_res = (int32_t)osThreadFlagsWait(2, 0, 10);
    if(os_res < 0){
        res = 1;
        goto end;
    }
end:
    MP6500_CS_GPIO_Port->BSRR = MP6500_CS_Pin;
    send_state = SEND_NONE;
    return res;
}

/**
 * @brief     interface delay ms
 * @param[in] ms
 * @note      none
 */
void mpu9250_interface_delay_ms(uint32_t ms)
{
    osDelay(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt is the format data
 * @note      none
 */
void mpu9250_interface_debug_print(const char *const fmt, ...)
{
    va_list vargs;
    va_start(vargs, fmt);
    shell_log(fmt, vargs);
}

/**
 * @brief     interface receive callback
 * @param[in] type is the irq type
 * @note      none
 */
void mpu9250_interface_receive_callback(uint8_t type)
{
    switch (type)
    {
        case MPU9250_INTERRUPT_MOTION :
        {
            mpu9250_interface_debug_print("mpu9250: irq motion.\n");
            
            break;
        }
        case MPU9250_INTERRUPT_FIFO_OVERFLOW :
        {
            mpu9250_interface_debug_print("mpu9250: irq fifo overflow.\n");
            
            break;
        }
        case MPU9250_INTERRUPT_FSYNC_INT :
        {
            mpu9250_interface_debug_print("mpu9250: irq fsync int.\n");
            
            break;
        }
        case MPU9250_INTERRUPT_DMP :
        {
            mpu9250_interface_debug_print("mpu9250: irq dmp\n");
            
            break;
        }
        case MPU9250_INTERRUPT_DATA_READY :
        {
            mpu9250_interface_debug_print("mpu9250: irq data ready\n");
            
            break;
        }
        default :
        {
            mpu9250_interface_debug_print("mpu9250: irq unknown code.\n");
            
            break;
        }
    }
}

/**
 * @brief     interface dmp tap callback
 * @param[in] count is the tap count
 * @param[in] direction is the tap direction
 * @note      none
 */
void mpu9250_interface_dmp_tap_callback(uint8_t count, uint8_t direction)
{
    switch (direction)
    {
        case MPU9250_DMP_TAP_X_UP :
        {
            mpu9250_interface_debug_print("mpu9250: tap irq x up with %d.\n", count);
            
            break;
        }
        case MPU9250_DMP_TAP_X_DOWN :
        {
            mpu9250_interface_debug_print("mpu9250: tap irq x down with %d.\n", count);
            
            break;
        }
        case MPU9250_DMP_TAP_Y_UP :
        {
            mpu9250_interface_debug_print("mpu9250: tap irq y up with %d.\n", count);
            
            break;
        }
        case MPU9250_DMP_TAP_Y_DOWN :
        {
            mpu9250_interface_debug_print("mpu9250: tap irq y down with %d.\n", count);
            
            break;
        }
        case MPU9250_DMP_TAP_Z_UP :
        {
            mpu9250_interface_debug_print("mpu9250: tap irq z up with %d.\n", count);
            
            break;
        }
        case MPU9250_DMP_TAP_Z_DOWN :
        {
            mpu9250_interface_debug_print("mpu9250: tap irq z down with %d.\n", count);
            
            break;
        }
        default :
        {
            mpu9250_interface_debug_print("mpu9250: tap irq unknown code.\n");
            
            break;
        }
    }
}

/**
 * @brief     interface dmp orient callback
 * @param[in] orientation is the dmp orientation
 * @note      none
 */
void mpu9250_interface_dmp_orient_callback(uint8_t orientation)
{
    switch (orientation)
    {
        case MPU9250_DMP_ORIENT_PORTRAIT :
        {
            mpu9250_interface_debug_print("mpu9250: orient irq portrait.\n");
            
            break;
        }
        case MPU9250_DMP_ORIENT_LANDSCAPE :
        {
            mpu9250_interface_debug_print("mpu9250: orient irq landscape.\n");
            
            break;
        }
        case MPU9250_DMP_ORIENT_REVERSE_PORTRAIT :
        {
            mpu9250_interface_debug_print("mpu9250: orient irq reverse portrait.\n");
            
            break;
        }
        case MPU9250_DMP_ORIENT_REVERSE_LANDSCAPE :
        {
            mpu9250_interface_debug_print("mpu9250: orient irq reverse landscape.\n");
            
            break;
        }
        default :
        {
            mpu9250_interface_debug_print("mpu9250: orient irq unknown code.\n");
            
            break;
        }
    }
}
