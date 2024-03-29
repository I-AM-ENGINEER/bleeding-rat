/*
    Данный пример демонстрирует управление двигателями с обратной связью по скорости
    скорость указыватется в метрах в секунду
*/

#include "core.h"

void core_init( void ){
	shell_log("start movement speed control example");

    // Настройки ПИД регулятора ООС по скорости
    // можно не вызывать, тогда будет инициализирован с настройками по умолчанию
    move_pid_speed_config( 0.002, 0.015, 0 );

    // Настройки ПИД регулятора синхронизации моторов
    // можно не вызывать, тогда будет инициализирован с настройками по умолчанию
    // Если синхронизация не нужна, можно указать все коэфиценты "0"
    move_pid_sync_config( 100, 0, 0 );

    // Ограничение максимальной мощности помогает при пробуксовке
    // если не нужно, можно не прописывать, по умолчанию ограничения нет
    move_max_power_set(0.8);

    // Разрешить движение (подать питание на двигатели)
	move_permit(true);
}

void core_loop( void ){
    // Установить скорость 100 мм/с на оба колеса
    move_speed_set(100, 100);
    delay(2000);
    
    // Остановка
    move_speed_set(0, 0);
    delay(500);

    // Установить скорость 200 мм/с на оба колеса в обратном направлении
    move_speed_set(-200, -200);
    delay(1000);

    // Остановка
    move_speed_set(0, 0);
    delay(3000);
    
    // Вращение против часовой стрелки, со скростью колес 100 мм/с
    move_speed_set(-100, 100);
    delay(500);

    // Остановка
    move_speed_set(0, 0);
    delay(500);
}