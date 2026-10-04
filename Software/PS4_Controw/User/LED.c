#include "stm32f4xx_hal.h"                  // Device header
#include "LED.h"
#include "gpio.h"

#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_1
#define LED3_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_0
#define LED4_GPIO_Port GPIOD

void LED1_ON(void)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
}

void LED1_OFF(void)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
}

void LED1_Turn(void)
{
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

void LED2_ON(void)
{
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
}

void LED2_OFF(void)
{
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
}

void LED2_Turn(void)
{
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
}

void LED3_ON(void)
{
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
}

void LED3_OFF(void)
{
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}

void LED3_Turn(void)
{
	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
}

void LED4_ON(void)
{
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
}

void LED4_OFF(void)
{
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
}

void LED4_Turn(void)
{
	HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
}
