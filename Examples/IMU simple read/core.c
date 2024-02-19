#include "core.h"

void core_init( void ){
    shell_log("start imu example");
}

void core_loop( void ){
    // Вектора, куда будут записаны данные об линейных и угловых ускорениях
    float a[3];
	float b[3];
	float t; // Переменная для считывания температуры IMU чипа

    // Считать вектора ускорений
	imu_read(a, g);
    // Считать температуру
	imu_read_temperature(&t);
    // Вывести значения в консоль
	shell_log("temp:%.1f\t\tacc:%.3f\t%.3f\t%.3f\t\tgyro:%.3f\t%.3f\t%.3f\t", t, a[0], a[1], a[2], b[0], b[1], b[2]);
}
