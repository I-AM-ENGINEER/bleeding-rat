#include "core.h"

void core_init( void ){
	shell_log("start movement speed control example");
    // надо добавить инициализацию ПИД
    // Разрешить движение (подать питание на двигатели)
	move_servos_permit(true);
}

void core_loop( void ){
    // Установить скорость 0.25 м/с на оба колеса
    move_servos_speed_set(0.25f, 0.25f);
    delay(2000);
    
    // Остановка
    move_servos_speed_set(0.0f, 0.0f);
    delay(500);

    // Установить скорость -1.0 м/с на оба колеса
    move_servos_speed_set(-1.0f, -1.0f);
    delay(500);

    // Остановка
    move_servos_speed_set(0.0f, 0.0f);
    delay(3000);
    
    // Вращение против часовой стрелки, со скростью колес 0.2 м/с
    move_servos_speed_set(-0.2f, 0.2f);
    delay(500);

    // Остановка
    move_servos_speed_set(0.0f, 0.0f);
    delay(500);

    // Вращение по часовой стрелке, со скростью колес 0.2 м/с
    move_servos_speed_set(0.2f, -0.2f);
    delay(500);

    // Остановка
    move_servos_speed_set(0.0f, 0.0f);
    delay(5000);
}