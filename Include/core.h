#ifndef CORE_H__
#define CORE_H__

#include "main.h"
#include "move.h"
#include "shell.h"
#include "system.h"
#include "imu.h"
#include "collision.h"
#include "ws2812b.h"

/// @brief Инициализация
void core_init( void );

/// @brief Основной цикл работы
void core_loop( void );

#endif // CORE_H__
