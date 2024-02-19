#include "core.h"

void core_init( void ){
	shell_log("start movement simple (power control) example");
    // Разрешить движение (подать питание на двигатели)
	move_servos_permit(true);
}

void core_loop( void ){
    float power = 0.0f;

    // Плавное ускорение до максимальной скорости моторов
    for(; power < 1.0f; i += 0.01f){
        move_servos_power_set(power, power);
        delay(5);
    }

    // Плавное замедление и ускорение в обратную сторону
    for(; power > -1.0f; i -= 0.01f){
        move_servos_power_set(power, power);
        delay(5);
    }

    // Плавное замедление до полной остановки
    for(; power < 0.0f; i += 0.01f){
        move_servos_power_set(power, power);
        delay(5);
    }

    delay(3000);
    // Вращение против часовой стрелки
    move_servos_power_set(-0.5f, 0.5f);
    delay(500);

    // Остановка
    move_servos_power_set(0.0f, 0.0f);
    delay(500);

    // Вращение по часовой стрелке
    move_servos_power_set(0.5f, -0.5f);
    delay(500);

    // Остановка
    move_servos_power_set(0.0f, 0.0f);
    delay(5000);
}