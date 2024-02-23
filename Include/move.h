// Low level motor driver

#ifndef MOVE_H__
#define MOVE_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "encoderd.h"
#include "motord.h"
#include "servo.h"
#include "PID.h"
#include "main.h"

/************************** Стандартные настройки, можно менять ***************************/

#define SERVO_DEFAULT_MAX_POWER             1.0f

#define SERVO_MINIMUM_RPM                   60.0f
#define SERVO_MAXIMUM_RPM                   1000.0f

#define SERVO_MINIMUM_DISTANCE              10.0f                   // Минимальная дистанция, на которую может сдвинуть сервопривод (мм)
#define WHEEL_DIAMETER                      12.0f                   // (мм), хз, надо проверить, линейки под рукой нет
#define ENCODER_STEPS_IN_ROTATION           12                      // Шагов на оборот

// Стандартные параметры ПИД ООС по скорости
#define SERVO_DEFAULT_PID_SPEED_KP          0.002f
#define SERVO_DEFAULT_PID_SPEED_KI          0.015f
#define SERVO_DEFAULT_PID_SPEED_KD          0.000f

// Стандартные параметры ПИД ООС по положению
#define SERVO_DEFAULT_PID_POSITION_KP       1500.0f
#define SERVO_DEFAULT_PID_POSITION_KI       0.0f
#define SERVO_DEFAULT_PID_POSITION_KD       0.0f

// Стандартные параметры ПИД синхронизации (если не надо, можно поставить все коэфиценты 0)
#define SERVO_DEFAULT_PID_SYNC_KP           100.0f
#define SERVO_DEFAULT_PID_SYNC_KI           0.0f
#define SERVO_DEFAULT_PID_SYNC_KD           0.0f
// Ведомый/ведущий синхронизации
#define SERVO_MASTER                        SERVO_LEFT             // Ведущий сервопривод, ведомый будет подстраиваться под него
#define SERVO_SLAVE                         SERVO_RIGHT            // По неизвестным мне причинам, препроцессор не могет в сравнение, надо вручную прописывать

// Вот честно, хз, зачем оно надо
#define SERVO_DEFAULT_PID_SPEED_TAU         1.0f
#define SERVO_DEFAULT_PID_POSITION_TAU      1.0f
#define SERVO_DEFAULT_PID_SYNC_TAU          1.0f


#define MOTOR_DEFAULT_DECAY                 MOTORD_DECAY_FAST

/*************** Аппаратные настройки, менять только при смене железа *****************/

#define SERVO_COUNT                         2

#define MOTOR_EN_GPIO                       MOT_PWR_EN_GPIO_Port
#define MOTOR_EN_PIN                        MOT_PWR_EN_Pin

#define MOTOR_R_TIM                         htim3
#define MOTOR_L_TIM                         htim3
#define MOTOR_R_PIN1_TIM_CHANNEL            TIM_CHANNEL_2
#define MOTOR_R_PIN2_TIM_CHANNEL            TIM_CHANNEL_1
#define MOTOR_L_PIN1_TIM_CHANNEL            TIM_CHANNEL_3
#define MOTOR_L_PIN2_TIM_CHANNEL            TIM_CHANNEL_4

#define ENCODER_L_PINA_PORT                 HALL3_EXTI_GPIO_Port
#define ENCODER_L_PINA_PIN                  HALL3_EXTI_Pin
#define ENCODER_L_PINB_PORT                 HALL2_EXTI_GPIO_Port
#define ENCODER_L_PINB_PIN                  HALL2_EXTI_Pin

#define ENCODER_R_PINA_PORT                 HALL4_EXTI_GPIO_Port
#define ENCODER_R_PINA_PIN                  HALL4_EXTI_Pin
#define ENCODER_R_PINB_PORT                 HALL1_EXTI_GPIO_Port
#define ENCODER_R_PINB_PIN                  HALL1_EXTI_Pin




typedef enum{
    SERVO_STATUS_MOVING,
    SERVO_STATUS_PARKED,
    SERVO_STATUS_DISABLES,
} servo_status_t;

typedef enum{
    MOVE_SYNC_NONE,
    MOVE_SYNC_SPEED,
    MOVE_SYNC_POSITION,
} move_sync_status_t;

typedef enum{
    SERVO_LEFT = 0,
    SERVO_RIGHT,
} servo_position_t;

struct servo_bundle_s{
    motord_t motord;
    encoderd_t encoderd;
    PIDController_t pid_speed;
    PIDController_t pid_distance;
    float rpm_target;
    float rotation_target;
    float rotation_start;
};

typedef struct{
    servo_t servo[SERVO_COUNT];
    struct servo_bundle_s servo_bundle[SERVO_COUNT];
    PIDController_t pid_distance_sync;
    move_sync_status_t sync_state;
    float sync_ratio_target;
    bool motion_permision;
} move_t;


/// @brief Инициализация движения
/// @return 0 в случае успешной инициализации, -1 в случае ошибки
int32_t move_init( void );

/// @brief Обработка ПИД регуляторов двигателей должна производиться с частотой 1000Гц
void move_process( void );

/// @brief Установка максимальной мощности двигателей
/// @param max_power Максимальная мощность (0.0...1.0)
void move_max_power_set( float max_power );

/// @brief Конфигурация ПИД регулятора скорости движения
/// @param kP Пропорциональный коэффициент
/// @param kI Интегральный коэффициент
/// @param kD Дифференциальный коэффициент
void move_pid_speed_config( float kP, float kI, float kD );

/// @brief Конфигурация ПИД регулятора позиции движения
/// @param kP Пропорциональный коэффициент
/// @param kI Интегральный коэффициент
/// @param kD Дифференциальный коэффициент
void move_pid_position_config( float kP, float kI, float kD );

/// @brief Конфигурация ПИД регулятора синхронизации двигателей
/// @param kP Пропорциональный коэффициент
/// @param kI Интегральный коэффициент
/// @param kD Дифференциальный коэффициент
void move_pid_sync_config( float kP, float kI, float kD );

/// @brief Разрешение или запрещение движения
/// @param permission Разрешение (true) или запрещение (false)
void move_permit( bool permission );

/// @brief Установка мощности двигателей
/// @param power_l Мощность левого двигателя (от -1 до 1)
/// @param power_r Мощность правого двигателя (от -1 до 1)
void move_power_set( float power_l, float power_r );

/// @brief Установка скорости двигателей
/// @param speed_l Скорость левого двигателя (в единицах измерения скорости)
/// @param speed_r Скорость правого двигателя (в единицах измерения скорости)
void move_speed_set( float speed_l, float speed_r );

/// @brief Установка позиции двигателей
/// @param position_l Позиция левого двигателя (в единицах измерения позиции)
/// @param position_r Позиция правого двигателя (в единицах измерения позиции)
/// @param max_speed Максимальная скорость движения (в единицах измерения скорости)
void move_position_set( float position_l, float position_r, float max_speed );

/// @brief Сброс позиции двигателей
void move_position_reset( void );

/// @brief Получение указателя на сервопривод по его позиции
/// @param servo_position Позиция сервопривода
/// @return Указатель на структуру сервопривода
servo_t *move_servo_get( servo_position_t servo_position );

/// @brief Обработка изменения состояния любого пина энкодеров
void move_encoders_process( void );

#endif // MOVE_H__
