#ifndef ENCODERD_H__
#define ENCODERD_H__

#include <stdint.h>
#include "stm32f4xx.h"

typedef struct{
    volatile int32_t steps;
    volatile uint8_t old_state;
    volatile uint16_t timer_period;
    uint16_t timer_period_filter[12];
    uint8_t timer_period_filter_i;

    GPIO_TypeDef *pin_a_gpio_port;
    uint16_t pin_a_pin;

    GPIO_TypeDef *pin_b_gpio_port;
    uint16_t pin_b_pin;

    TIM_HandleTypeDef *htim_period;
} encoderd_t;

/// @brief Функция инициализации инкриментального энкодера
/// @param encoderd указатель на инициализируемый энкодер
/// @param pin_a_gpio_port порт, к которому подключен пин A энкодера
/// @param pin_a_pin пин, к которому подключен вывод A энкодера
/// @param pin_b_gpio_port порт, к которому подключен пин B энкодера
/// @param pin_b_pin пин, к которому подключен пин B энкодера
/// @param htim_period (опционально) 16 битный таймер для измерения частоты вращения энкодера. Один энкодер = 1 таймер
/// @return 0 - инициализация успешна, -1 - ошибка
int encoderd_init( encoderd_t *encoderd,\
    GPIO_TypeDef *pin_a_gpio_port, uint16_t pin_a_pin,\
    GPIO_TypeDef *pin_b_gpio_port, uint16_t pin_b_pin,\
    TIM_HandleTypeDef *htim_period\
);

/// @brief Вызывать при изменении состояний выводов A или B
/// @param encoderd Указатель на обрабатываемый энкодер
/// @return 0 - Успешная обработка, -1 - ошибка, -2 - пропуск шага
int32_t encoderd_process( encoderd_t *encoderd );

/// @brief Вызывать при переполнении таймера, используемого для определениия частоты вращения. Необходимо для определения нулевой скорости
/// @param encoderd Указатель на обрабатываемый энкодер
void encoderd_period_timer_overflow_irq( encoderd_t *encoderd );

/// @brief Установить текущее положение энкодера как нулевое
/// @param encoderd 
void encoderd_reset( encoderd_t *encoderd );

/// @brief Считать кол-во шагов энкодера с момента инициализации/сброса нулевого положения
/// @param encoderd Указатель на обрабатываемый энкодер
/// @return Кол-во шагов с момента инициализации/сброса нулевого положения
int32_t encoderd_get_steps( encoderd_t *encoderd );

/// @brief Считать скорость вращения энкодера. Работает только если энкодеру присвоен таймера при инициализации
/// @param encoderd Указатель на обрабатываемый энкодер
/// @return Количество шагов в секунду
float encoderd_get_steps_per_second( encoderd_t *encoderd );

#endif // ENCODERD_H__