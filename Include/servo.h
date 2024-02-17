#ifndef SERVO_H__
#define SERVO_H__

#include <stdint.h>
#include "motord.h"
#include "encoderd.h"
#include "PID.h"

typedef enum{
    SERVO_MODE_NO_FEEDBACK,
    SERVO_MODE_SPEED_FEEDBACK,
    SERVO_MODE_POSITION_FEEDBACK,
} servo_mode_t;

typedef struct{
    motord_t *motord;
    encoderd_t *encoderd;
    PIDController_t *pid_position;
    PIDController_t *pid_rpm;

    uint32_t steps_per_rotate;

    float max_power;
    float power;
    float target_postion;
    float position;
    float target_rpm;
    float rpm;

    servo_mode_t mode;
} servo_t;

/// @brief Инициализация структуры сесерводвигателя
/// @param servo Указатель на структуру серводвигателя
/// @param motor Указатель на структуру тотора (структура должна существовать все время жизни сервопривода)
/// @param encoder Указатель на структуру энкодера (структура должна существовать все время жизни сервопривода)
/// @param steps_per_rotate Количество шагов энкодера на полный оборот
/// @param pid_rpm  Указатель на структуру ПИД регулятора для управления скоростью вращения (структура должна существовать все время жизни сервопривода). Если не нужно, можно указать NULL, однако требуется для управления позицией
/// @param pid_position Указатель на структуру ПИД регулятора для управления положением (структура должна существовать все время жизни сервопривода). Если не нужно, можно указать NULL, для работы требуется наличие pid_rpm
/// @return 0 - успешная инициализация, любое другое значение - ошибка
int32_t servo_init( servo_t *servo, motord_t *motor, encoderd_t *encoder, uint32_t steps_per_rotate, PIDController_t *pid_rpm, PIDController_t *pid_position );

/// @brief Установить режим работы
/// @param servo Указатель на структуру серводвигателя
/// @param mode SERVO_MODE_NO_FEEDBACK - без обратной связи, управление мощностью, SERVO_MODE_SPEED_FEEDBACK - обратная связь по скорости, стремится к заданной частоте вражения, SERVO_MODE_POSITION_FEEDBACK - обратная связь по скорости и положению, стремится к заданному положению
/// @return 0 - успешная установка, любое другое значение - ошибка
int32_t servo_mode_set( servo_t *servo, servo_mode_t mode );

/// @brief Если установлен режим без обратной связи (SERVO_MODE_NO_FEEDBACK), управляет мощностью двигателя 
/// @param servo Указатель на структуру серводвигателя
/// @param power Требуемая мощность двигателя (-1.0...1.0)
/// @return 0 - успешная установка, любое другое значение - ошибка
int32_t servo_power_set( servo_t *servo, float power );

/// @brief Возвращает текущую мощность двигателя
/// @param servo Указатель на структуру серводвигателя
/// @return мощность двигателя (-1.0...1.0)
float servo_power_get( servo_t *servo );

/// @brief Если установлен режим с обратной связью по скорости (SERVO_MODE_SPEED_FEEDBACK), стремится поддержать требуемую частоту вращения
/// @param servo Указатель на структуру серводвигателя
/// @param rpm Требуемая частота вращения (оборотов в минуту)
/// @param max_power Максимальная мощность двигателя (-1.0...1.0)
/// @return 0 - успешная установка, любое другое значение - ошибка
int32_t servo_rpm_set( servo_t *servo, float rpm, float max_power );

/// @brief Возвращает текущую частоту вращения
/// @param servo Указатель на структуру серводвигателя
/// @return Частота вращения (оборотов в минуту)
float servo_rpm_get( servo_t *servo );

/// @brief Если установлен режим с обратной связью по скорости и положения (SERVO_MODE_POSITION_FEEDBACK), стремится занять и поддерживать указанное положение
/// @param servo Указатель на структуру серводвигателя
/// @param position Требуемая позиция серводвигателя (кол-во оборотов)
/// @param max_speed Максимально допустимая скорость (оборотов в минуту)
/// @param max_power Максимальная мощность двигателя (-1.0...1.0)
/// @return 0 - успешная установка, любое другое значение - ошибка
int32_t servo_position_set( servo_t *servo, float position, float max_speed, float max_power );

/// @brief Возвращает текущее положение серводвигателя
/// @param servo Указатель на структуру серводвигателя
/// @return Позиция серводвигателя (кол-во оборотов)
float servo_position_get( servo_t *servo );

/// @brief Установить нулевую позицию серводвигателя
/// @param servo Указатель на структуру серводвигателя
/// @return 0 - успешная установка, любое другое значение - ошибка
int32_t servo_position_reset( servo_t *servo );

/// @brief Обработка ПИД регуляторов, требуется вызывать с частотой 100...1000Гц для нормальной работы
/// @param servo Указатель на структуру серводвигателя
/// @return 0 - успешная установка, любое другое значение - ошибка
int32_t servo_process( servo_t *servo );

#endif // !SERVO_H__
