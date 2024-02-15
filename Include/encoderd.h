#ifndef ENCODERD_H__
#define ENCODERD_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"

typedef struct{
    volatile int32_t steps;
    volatile uint8_t old_state;

    GPIO_TypeDef *pin_a_gpio_port;
    uint16_t pin_a_pin;

    GPIO_TypeDef *pin_b_gpio_port;
    uint16_t pin_b_pin;
} encoderd_t;

int encoderd_init( encoderd_t *encoderd,\
    GPIO_TypeDef *pin_a_gpio_port, uint16_t pin_a_pin,\
    GPIO_TypeDef *pin_b_gpio_port, uint16_t pin_b_pin\
);

int32_t encoderd_process( encoderd_t *encoderd );
void encoderd_reset( encoderd_t *encoderd );
int32_t encoderd_get_steps( encoderd_t *encoderd );

#endif // ENCODERD_H__