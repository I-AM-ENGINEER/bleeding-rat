#ifndef ENCODERD_H__
#define ENCODERD_H__

#include <stdint.h>
#include "stm32f4xx.h"
// Only for time_us()
#include "system.h"

#define RPM_FILTER_K 0.3 // Exponential smoothing factor

#define ENCODERD_US_FUNCTION()      time_us()

typedef struct{
    volatile int32_t steps;
    volatile uint8_t old_state;
    volatile uint16_t timer_period;

    GPIO_TypeDef *pin_a_gpio_port;
    uint16_t pin_a_pin;

    GPIO_TypeDef *pin_b_gpio_port;
    uint16_t pin_b_pin;
    
    uint32_t max_delta_time;
    volatile float filter_steps_per_second;
    volatile uint64_t timestamp_us_last;
} encoderd_t;

/// @brief Функция инициализации инкриментального энкодера
/// @param encoderd Указатель на инициализируемый энкодер
/// @param steps_per_second_min Минимальное кол-во шагов в секунду, фиксируемое энкодером как движениие
/// @param pin_a_gpio_port Порт, к которому подключен пин A энкодера
/// @param pin_a_pin Пин, к которому подключен вывод A энкодера
/// @param pin_b_gpio_port Порт, к которому подключен пин B энкодера
/// @param pin_b_pin Пин, к которому подключен пин B энкодера
/// @return 0 - инициализация успешна, -1 - ошибка
int32_t encoderd_init( encoderd_t *encoderd,\
    float steps_per_second_min,\
    GPIO_TypeDef *pin_a_gpio_port, uint16_t pin_a_pin,\
    GPIO_TypeDef *pin_b_gpio_port, uint16_t pin_b_pin\
);

/// @brief Вызывать при изменении состояний выводов A или B
/// @param encoderd Указатель на обрабатываемый энкодер
/// @return 0 - Успешная обработка, -1 - ошибка, -2 - пропуск шага
int32_t encoderd_process_isr( encoderd_t *encoderd );

/// @brief Вызывать с переодичностью 10-1000Гц, в зависимости от требований к реакции на изменение скорости
/// @param encoderd Указатель на обрабатываемый энкодер
/// @return 0 - Успешная обработка, -1 - ошибка, -2 - пропуск шага
int32_t encoderd_process_rpm( encoderd_t *encoderd );

/// @brief Установить текущее положение энкодера как нулевое
/// @param encoderd 
int32_t encoderd_reset( encoderd_t *encoderd );

/// @brief Считать кол-во шагов энкодера с момента инициализации/сброса нулевого положения
/// @param encoderd Указатель на обрабатываемый энкодер
/// @return Кол-во шагов с момента инициализации/сброса нулевого положения
int32_t encoderd_get_steps( encoderd_t *encoderd );

/// @brief Считать скорость вращения энкодера. Работает только если энкодеру присвоен таймера при инициализации
/// @param encoderd Указатель на обрабатываемый энкодер
/// @return Количество шагов в секунду
float encoderd_get_steps_per_second( encoderd_t *encoderd );

#endif // ENCODERD_H__
