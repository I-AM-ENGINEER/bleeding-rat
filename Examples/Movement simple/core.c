/*
    Данный пример демонстрирует работу с двигателями без обратных связей, самый простой способ
    управления двигателями, мощность двигателя задается в пределах -1.0...1.0, где
         1.0 - полная мощность, движение вперед
        -1.0 - полная мощность, движение назад
         0.0 - остановка
    В функцию управления передаются 2 скорости: мощность для левого и правого двигателя, соответственно
*/

#include "core.h"

void core_init( void ){
	shell_log("start movement simple (power control) example");
    // Разрешить движение (подать питание на двигатели)
	move_servos_permit(true);
}

void core_loop( void ){
    // Остановка
    move_servos_power_set(0.0, 0.0);
    delay(500);

    // Движение вперед
    move_servos_power_set(0.6, 0.6);
    delay(500);

    // Остановка
    move_servos_power_set(0.0, 0.0);
    delay(500);

    // Вращение против часовой стрелки
    move_servos_power_set(-0.6, 0.6);
    delay(300);
}