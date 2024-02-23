#ifndef SHELL_H__
#define SHELL_H__

#define SHELL_TIME_US_FUNCTION  time_us
#define SHELL_UART              huart1

#include <stdint.h>
#include <stdlib.h>

/// @brief Инициализация оболочки (shell)
/// @return 0 в случае успешной инициализации, -1 в случае ошибки
int32_t shell_init( void );

/// @brief Отправка массива данных через оболочку (shell)
/// @param buff Указатель на массив данных для отправки
/// @param length Длина массива данных
void shell_send_arr( const uint8_t* buff, size_t length );

/// @brief Логирование сообщения через оболочку (shell)
/// @param format Строка формата для логирования
/// @param ... Аргументы, подставляемые в строку формата
void shell_log( const char *format, ... );

#endif // SHELL_H__
