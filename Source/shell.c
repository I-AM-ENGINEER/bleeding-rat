#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "stm32f4xx.h"
#include "shell.h"
#include "lwshell/lwshell.h"
#include "lwrb/lwrb.h"
#include "help.h"
#include "system.h"

extern UART_HandleTypeDef SHELL_UART;

static uint8_t rx_buffer_dat[4096];
static uint8_t tx_buffer_dat[4096];

static uint8_t dma_tx_buff[256];
static uint8_t dma_rx_buff[256];

static lwrb_t tx_buffer;
static lwrb_t rx_buffer;

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

void HAL_UARTEx_RxEventCallback( UART_HandleTypeDef *huart, uint16_t Size ){
    if(huart->Instance == SHELL_UART.Instance){
        HAL_UARTEx_ReceiveToIdle_DMA(&SHELL_UART, dma_rx_buff, sizeof(dma_rx_buff));
        shell_send_arr(dma_rx_buff, Size);
        uint16_t writen_byted = lwrb_write(&rx_buffer, dma_rx_buff, Size);
        if(writen_byted != Size){
            shell_log("[shell]: rx buffer overflow");
        }
    }
}

void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart ){
    if(huart->Instance == SHELL_UART.Instance){
        shell_send_buffer();
    }
}

int fputc(int ch, FILE *f){
    shell_send_arr((uint8_t*)&ch, 1);
    return HAL_OK;
}

void shell_output_fn( const char* str, struct lwshell* lwobj ){
    shell_send_arr((const uint8_t*)str, strlen(str));
}

int shell_help( int32_t argc, char** argv ){
    shell_send_arr((const uint8_t*)help_text, strlen(help_text));
    return 0;
}

int32_t shell_init( void ){
    lwshell_init();
    lwshell_set_output_fn(shell_output_fn);
    lwrb_init(&rx_buffer, rx_buffer_dat, sizeof(rx_buffer_dat));
    lwrb_init(&tx_buffer, tx_buffer_dat, sizeof(tx_buffer_dat));
    lwshell_register_cmd("help", shell_help, "help function");
    HAL_UARTEx_ReceiveToIdle_DMA(&SHELL_UART, (uint8_t*)dma_rx_buff, sizeof(dma_rx_buff));
    return 0;
}

void shell_log( const char *format, ... ){
    va_list vargs;
    va_start(vargs, format);
    uint64_t tick = SHELL_TIME_US_FUNCTION();
    char time_str[20];
    uint32_t us = tick%1000000;
    uint32_t s  = tick/1000000;
    printf("[%6u.%.6u] ", s, us);
    vprintf(format, vargs);
    printf("\r\n");
    va_end(vargs);
}

void shell_backspace_process ( char *str ) {
    char *src, *dst;
    for (src = dst = str; *src != '\0'; src++) {
        if (*src == 0x7F) {
            if (dst != str) {
                dst--;
            }
            continue;
        }
        *dst++ = *src;
    }
    *dst = '\0';
}

void shell_process( void ){
    static size_t checked_bytes;
    static char last_character = '\n'; // \r\n double print ">>" protection
    char buff[16];
    size_t buff_len;
    do{
        buff_len = lwrb_peek(&rx_buffer, checked_bytes, buff, sizeof(buff));
        if(buff_len > 0){
            for(uint32_t i = 0; i < buff_len; i++){
                if((buff[i] == '\r') || (buff[i] == '\n')){
                    char current_byte;
                    lwrb_read(&rx_buffer, &current_byte, 1);
                    size_t readed_len = lwrb_read(&rx_buffer, buff, checked_bytes+i);
                    checked_bytes = 0;
                    if((last_character == '\n') && (current_byte == '\r')){
                        continue;
                    }
                    last_character = current_byte;
                    buff[readed_len] = '\0';
                    shell_backspace_process(buff);
                    lwshell_input(&buff, strlen(buff));
                    printf(">>");
                }else{
                    checked_bytes++;
                    if(checked_bytes == sizeof(buff)){
                        checked_bytes--;
                        lwrb_skip(&rx_buffer, 1);
                        continue;
                    }
                }
            }
        }
    }while (buff_len != 0);
}