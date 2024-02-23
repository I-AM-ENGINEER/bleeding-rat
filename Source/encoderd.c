#include "encoderd.h"

const int8_t lookup_table[4][4] = {
    {0, -1, 1, 2},
    {1, 0, 2, -1},
    {-1, 2, 0, 1},
    {2, 1, -1, 0},
};

int32_t encoderd_init( encoderd_t *encoderd,\
    float steps_per_second_min,\
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
    
    float us_per_step_max = (1.0f/steps_per_second_min) * 1E6f;

    encoderd->pin_a_gpio_port = pin_a_gpio_port;
    encoderd->pin_b_gpio_port = pin_b_gpio_port;
    encoderd->pin_a_pin = pin_a_pin;
    encoderd->pin_b_pin = pin_b_pin;
    encoderd->steps = 0;
    encoderd->filter_steps_per_second = 0.0f;
    encoderd->max_delta_time = (uint32_t)us_per_step_max;
    return 0;
}

float moving_exponential_mean_filter(float new_value, float prev_filtered_value) {
    return (RPM_FILTER_K * new_value) + ((1.0f - RPM_FILTER_K) * prev_filtered_value);
}

int32_t encoderd_process_isr( encoderd_t *encoderd ){
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

    uint32_t pin_a_state = HAL_GPIO_ReadPin(encoderd->pin_a_gpio_port, encoderd->pin_a_pin) == GPIO_PIN_SET ? 1 : 0;
    uint32_t pin_b_state = HAL_GPIO_ReadPin(encoderd->pin_b_gpio_port, encoderd->pin_b_pin) == GPIO_PIN_SET ? 1 : 0;
    uint32_t state = pin_a_state + (pin_b_state << 1);

    int8_t delta = lookup_table[state][encoderd->old_state];

    if(delta == 2){
        res = -2;
    }else if(delta != 0){
        encoderd->steps += (int32_t)delta;

        uint64_t timestamp_current = ENCODERD_US_FUNCTION();
        uint64_t timestamp_last = encoderd->timestamp_us_last;

        uint32_t time_us_delta = (uint32_t)(timestamp_current - timestamp_last);

        float steps_per_s = (float)delta * 1E6f / (float)time_us_delta;


        float steps_per_s_last = encoderd->filter_steps_per_second;

        float new_fiter_value = moving_exponential_mean_filter(steps_per_s, steps_per_s_last);
        
        encoderd->filter_steps_per_second = new_fiter_value;
        encoderd->timestamp_us_last = timestamp_current;
    }
    
    encoderd->old_state = state;

    return res;
}

int32_t encoderd_reset( encoderd_t *encoderd ){
    if(encoderd == NULL){
        return -1;
    }

    encoderd->steps = 0;

    return 0;
}

int32_t encoderd_get_steps( encoderd_t *encoderd ){
    if(encoderd == NULL){
        return 0;
    }

    return encoderd->steps;
}

int32_t encoderd_process_rpm( encoderd_t *encoderd ){
    if(encoderd == NULL){
        return -1;
    }

    uint64_t timestamp_current = ENCODERD_US_FUNCTION();
    uint64_t timestamp_last = encoderd->timestamp_us_last;

    uint32_t time_us_delta = (uint32_t)(timestamp_current - timestamp_last);

    if(time_us_delta > encoderd->max_delta_time){
        encoderd->filter_steps_per_second = 0;
    }

    return 0;
}

float encoderd_get_steps_per_second( encoderd_t *encoderd ){
    if(encoderd == NULL){
        return 0.0f;
    }

    float sps = encoderd->filter_steps_per_second;
    return sps;
}
