#ifndef SHELL_H__
#define SHELL_H__

#define SHELL_TIME_US_FUNCTION  time_us
#define SHELL_UART              huart1

#include <stdint.h>
#include <stdlib.h>

int32_t shell_init( void );
void shell_process( void );
void shell_send_arr( const uint8_t* buff, size_t length );
void shell_log( const char *format, ... );

#endif // SHELL_H__
