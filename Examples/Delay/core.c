/*
    Демонстрация работы функций задержек. Всего их две:
        - delay()     задержка в миллисекундах
        - delay_us()  задержка в микросекундах
*/
#include "core.h"

void core_init( void ){
    // Нечего инициализировать
}

void core_loop( void ){
    // Измеряем время запуска функции задержки
    uint64_t delay_us_start_time = time_us();
    delay_us(1337);
    // Измеряем время выхода из функции задержки
    uint64_t delay_us_stop_time = time_us();
    // Считаем проведенное время в задержке
    uint32_t delay_us_delta = (uint32_t)(delay_us_stop_time - delay_us_start_time);
    shell_log("Delay 1337us is %luus long", delay_us_delta);

    // Измеряем время запуска функции задержки
    uint64_t delay_ms_start_time = time_us();
    delay(666);
    // Измеряем время выхода из функции задержки
    uint64_t delay_ms_stop_time = time_us();
    // Считаем проведенное время в задержке
    uint32_t delay_ms_delta = (uint32_t)(delay_ms_stop_time - delay_ms_start_time);
    shell_log("Delay 666ms is %luus long", delay_ms_delta);
    // Видно, что функция delay() имеет погрешность, в реальности погрешность составляет +-1мс
}
