#include "core.h"

void core_init( void ){
	shell_log("start movement speed control example");
    // надо добавить инициализацию ПИД
    // надо добавить инициализацию ещё одного ПИД
    // Разрешить движение (подать питание на двигатели)
	move_servos_permit(true);

    // Сброс текущего положеня
    move_servos_position_reset();
}

void core_loop( void ){
    // Установить скорость 0.25 м/с на оба колеса
    move_servos_speed_set(0.25f, 0.25f);
    delay(2000);
}