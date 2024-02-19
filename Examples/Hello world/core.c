#include "core.h"

void core_init( void ){
    shell_log("start hello world example");
}

void core_loop( void ){
    // Вывод сообщения с пометкой времени, удобно для системных сообщений
    shell_log("hello world with shell timestamp");
    // Ожидание 500мс
    delay(500);
    // Вывод сообщения без пометки времени, "\r\n" - перенос строки
    printf("hello world with standard output\r\n");
    // Ожидание 500мс
    delay(500);
}
