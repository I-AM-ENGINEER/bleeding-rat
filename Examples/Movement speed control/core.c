/*
    Данный пример демонстрирует управление двигателями с обратной связью по скорости
    скорость указыватется в метрах в секунду
*/

#include "core.h"

void core_init( void ){
	shell_log("start movement speed control example");
    // надо добавить инициализацию ПИД
    // Разрешить движение (подать питание на двигатели)
	move_servos_permit(true);
}

void core_loop( void ){
    // Установить скорость 100 мм/с на оба колеса
    move_servos_speed_set(100, 100);
    delay(2000);
    
    // Остановка
    move_servos_speed_set(0, 0);
    delay(500);

    // Установить скорость 200 мм/с на оба колеса в обратном направлении
    move_servos_speed_set(-200, -200);
    delay(1000);

    // Остановка
    move_servos_speed_set(0, 0);
    delay(3000);
    
    // Вращение против часовой стрелки, со скростью колес 100 мм/с
    move_servos_speed_set(-100, 100);
    delay(500);

    // Остановка
    move_servos_speed_set(0, 0);
    delay(500);
}