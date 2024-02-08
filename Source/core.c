#include "core.h"
#include "shell.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim7;

void core_init( void ){
	HAL_TIM_Base_Start(&htim7); // Timer for us ticks
	shell_init();
}

void core_loop( void ){
	shell_process();
	//HAL_UART_Transmit()
	//HAL_GPIO_WritePin(MOT_PWR_EN_GPIO_Port, MOT_PWR_EN_Pin, GPIO_PIN_SET);
	//HAL_Delay(100);
	//HAL_GPIO_WritePin(MOT_PWR_EN_GPIO_Port, MOT_PWR_EN_Pin, GPIO_PIN_RESET);
}
