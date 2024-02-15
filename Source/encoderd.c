#include "encoderd.h"
#include "shell.h"

const int8_t lookup_table[4][4] = {
    {0, -1, 1, 2},
    {1, 0, 2, -1},
    {-1, 2, 0, 1},
    {2, 1, -1, 0},
};

int encoderd_init( encoderd_t *encoderd,\
    GPIO_TypeDef *pin_a_gpio_port, uint16_t pin_a_pin,\
    GPIO_TypeDef *pin_b_gpio_port, uint16_t pin_b_pin)
{
    if(encoderd == NULL){
        return -1;
    }
    if(pin_a_gpio_port == NULL){
        return -1;
    }
    if(pin_b_gpio_port == NULL){
        return -1;
    }
    encoderd->pin_a_gpio_port = pin_a_gpio_port;
    encoderd->pin_b_gpio_port = pin_b_gpio_port;
    encoderd->pin_a_pin = pin_a_pin;
    encoderd->pin_b_pin = pin_b_pin;
    encoderd->steps = 0;
    return 0;
}

int32_t encoderd_process( encoderd_t *encoderd ){
    if(encoderd == NULL){
        return -1;
    }
    if(encoderd->pin_a_gpio_port == NULL){
        return -1;
    }
    if(encoderd->pin_b_gpio_port == NULL){
        return -1;
    }
    int32_t res = 0;

    uint8_t pin_a_state = HAL_GPIO_ReadPin(encoderd->pin_a_gpio_port, encoderd->pin_a_pin) == GPIO_PIN_SET ? 1 : 0;
    uint8_t pin_b_state = HAL_GPIO_ReadPin(encoderd->pin_b_gpio_port, encoderd->pin_b_pin) == GPIO_PIN_SET ? 1 : 0;
    uint8_t state = pin_a_state + (pin_b_state << 1);

    int8_t delta = lookup_table[state][encoderd->old_state];

    if(delta == 2){
        res = -2;
    }else{
        encoderd->steps += (int32_t)delta;
    }

    encoderd->old_state = state;
    return res;
}

void encoderd_reset( encoderd_t *encoderd ){
    encoderd->steps = 0;
}

int32_t encoderd_get_steps( encoderd_t *encoderd ){
    return encoderd->steps;
}
