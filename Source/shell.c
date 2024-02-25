#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "stm32f4xx.h"
#include "shell.h"
#include "lwrb/lwrb.h"
#include "system.h"

extern UART_HandleTypeDef SHELL_UART;

static uint8_t tx_buffer_dat[4096];
static uint8_t dma_tx_buff[256];
static lwrb_t tx_buffer;

#ifdef __GNUC__
int _write(int fd, char * ptr, int len){
  shell_send_arr((uint8_t*)ptr, len);
  return len;
}
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f){
  shell_send_arr((uint8_t*)&ch, 1);
  return HAL_OK;
}
#endif /* __GNUC__ */

void shell_send_buffer( void ){
    if(SHELL_UART.gState == HAL_UART_STATE_READY){
        size_t tx_size = lwrb_read(&tx_buffer, dma_tx_buff, sizeof(dma_tx_buff));
        HAL_UART_Transmit_DMA(&SHELL_UART, dma_tx_buff, tx_size);
    }
}

void shell_send_arr( const uint8_t* buff, size_t length ){
    lwrb_write(&tx_buffer, buff, length);
    shell_send_buffer();
}

int32_t shell_init( void ){
    //lwrb_init(&rx_buffer, rx_buffer_dat, sizeof(rx_buffer_dat));
    lwrb_init(&tx_buffer, tx_buffer_dat, sizeof(tx_buffer_dat));
    //HAL_UARTEx_ReceiveToIdle_DMA(&SHELL_UART, (uint8_t*)dma_rx_buff, sizeof(dma_rx_buff));
    return 0;
}

void shell_log( const char *format, ... ){
    va_list vargs;
    va_start(vargs, format);
    uint64_t tick = SHELL_TIME_US_FUNCTION();
    uint32_t us = tick%1000000;
    uint32_t s  = tick/1000000;
    // Keil compiler mark as warning, false alarm
    printf("[%6lu.%.6lu] ", s, us);
    vprintf(format, vargs);
    printf("\r\n");
    va_end(vargs);
}

void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart ){
    if(huart->Instance == SHELL_UART.Instance){
        shell_send_buffer();
    }
}

// В данный момент прием данных невозможен
/*
void HAL_UARTEx_RxEventCallback( UART_HandleTypeDef *huart, uint16_t Size ){
    if(huart->Instance == SHELL_UART.Instance){
        HAL_UARTEx_ReceiveToIdle_DMA(&SHELL_UART, dma_rx_buff, sizeof(dma_rx_buff));
        shell_send_arr(dma_rx_buff, Size);
        uint16_t writen_byted = lwrb_write(&rx_buffer, dma_rx_buff, Size);
        if(writen_byted != Size){
            shell_log("[shell]: rx buffer overflow");
        }
    }
}*/
