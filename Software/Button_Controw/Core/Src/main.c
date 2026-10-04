/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LED.h"
#include "KEY.h"
#include "OLED.h"
#include "MyCAN.h"
#include "smd.h"
#include "process_frame.h"
#include "Stepper_Arm.h"
#include "End_Effector.h"
#include "TTL_Servo.h"
#include "string.h"
#include "stdio.h"
#include <limits.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int32_t position1 = 0;
float angle1 = 0;
int32_t position2 = 0;
float angle2 = 0;
int32_t position3 = 0;
float angle3 = 0;

float x = 0;
float y = 0;
float z = 0;

/* Mode selection */
uint8_t control_mode = 0;      // 0: Angle mode, 1: 3D mode
uint8_t selected_axis = 0;     // 0: no selection, 1: axis1/x, 2: axis2/y, 3: axis3/z

/* Target values */
float target_angle[4] = {0, 0, 0, 0};   // index 1..3
float target_pos[3] = {0, 0, 0};        // x, y, z

/* Error flags */
uint8_t target_dead_area = 0;
uint8_t current_dead_area = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	MyCAN_Init();
	TTL_Servo_Init();
  EndEffector_Init();
  OLED_Update();
	arm_enable();
	HAL_TIM_Base_Start_IT(&htim6);
	
	uint32_t start_wait = HAL_GetTick();
	while ((!motors[1].data_ready || !motors[2].data_ready || !motors[3].data_ready) &&
       (HAL_GetTick() - start_wait < 500))
	{
    HAL_Delay(10);
	}
	// Read initial angles and clear data_ready flags
	if (motors[1].data_ready) { angle1 = motors[1].angle; motors[1].data_ready = 0; }
	if (motors[2].data_ready) { angle2 = motors[2].angle; motors[2].data_ready = 0; }
	if (motors[3].data_ready) { angle3 = motors[3].angle; motors[3].data_ready = 0; }
	get_current_position(&x, &y, &z);

	// Initialize target values (rounded to nearest 10 degrees / 20 mm)
	target_angle[1] = roundf(angle1 / 10.0f) * 10.0f;
	target_angle[2] = roundf(angle2 / 10.0f) * 10.0f;
	target_angle[3] = roundf(angle3 / 10.0f) * 10.0f;
	target_pos[0] = roundf(x / 20.0f) * 20.0f;
	target_pos[1] = roundf(y / 20.0f) * 20.0f;
	target_pos[2] = roundf(z / 20.0f) * 20.0f;

	// Set default mode and axis selection
	control_mode = 0;       // Angle mode
	selected_axis = 0;      // No axis selected
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (motors[1].data_ready) {
			position1 = motors[1].position;
			angle1 = motors[1].angle;
			motors[1].data_ready = 0;
		}
		
		if (motors[2].data_ready) {
			position2 = motors[2].position;
			angle2 = motors[2].angle;
			motors[2].data_ready = 0;
		}
		
		if (motors[3].data_ready) {
			position3 = motors[3].position;
			angle3 = motors[3].angle;
			motors[3].data_ready = 0;
		}
		
		get_current_position(&x, &y, &z);
		
		if (!dead_area_angle(angle1, angle2, angle3)) {
			if (!current_dead_area) {
				current_dead_area = 1;
				arm_disable();
				OLED_ClearArea(0, 56, 128, 8);
				OLED_ShowString(0, 56, "Error: Current DA", OLED_6X8);
				OLED_Update();
        HAL_Delay(3000);
        OLED_ClearArea(0, 56, 128, 8);
        }
    } else {
				current_dead_area = 0;
    }
		
		OLED_ClearArea(0, 0, 128, 56);
		OLED_ShowString(0, 0, "P1:", OLED_6X8);
		OLED_ShowString(0, 8, "A1:", OLED_6X8);
		OLED_ShowSignedNum(18, 0, position1, 7, OLED_6X8);
		OLED_ShowFloatNum(18, 8, angle1, 3, 2, OLED_6X8);
		OLED_ShowString(0, 16, "P2:", OLED_6X8);
		OLED_ShowString(0, 24, "A2:", OLED_6X8);
		OLED_ShowSignedNum(18, 16, position2, 7, OLED_6X8);
		OLED_ShowFloatNum(18, 24, angle2, 3, 2, OLED_6X8);
		OLED_ShowString(0, 32, "P3:", OLED_6X8);
		OLED_ShowString(0, 40, "A3:", OLED_6X8);
		OLED_ShowSignedNum(18, 32, position3, 7, OLED_6X8);
		OLED_ShowFloatNum(18, 40, angle3, 3, 2, OLED_6X8);
		
		OLED_ShowString(72, 0, "X:", OLED_6X8);
		OLED_ShowFloatNum(84, 0, x, 3, 2, OLED_6X8);
		OLED_ShowString(72, 8, "Y:", OLED_6X8);
		OLED_ShowFloatNum(84, 8, y, 3, 2, OLED_6X8);
		OLED_ShowString(72, 16, "Z:", OLED_6X8);
		OLED_ShowFloatNum(84, 16, z, 3, 2, OLED_6X8);
		
    if (control_mode == 0) {
			OLED_ShowString(72, 32, "Mode:Ang", OLED_6X8);
    } else {
			OLED_ShowString(72, 32, "Mode:3D   ", OLED_6X8);
    }
		
    if (selected_axis == 0) {
			OLED_ShowString(72, 40, "Axis:None ", OLED_6X8);
		} else {
			if (control_mode == 0) {
				char buf[16];
				sprintf(buf, "Axis:%d     ", selected_axis);
        OLED_ShowString(72, 40, buf, OLED_6X8);
      } else {
				char buf[16];
        sprintf(buf, "Coord:%c     ", (selected_axis == 1 ? 'X' : (selected_axis == 2 ? 'Y' : 'Z')));
        OLED_ShowString(72, 40, buf, OLED_6X8);
			}
		}
		
    OLED_ShowString(0, 48, "Target:", OLED_6X8);
    if (control_mode == 0) {
			if (selected_axis == 0) {
				OLED_ShowFloatNum(42, 48, target_angle[1], 3, 0, OLED_6X8);
				OLED_ShowFloatNum(66, 48, target_angle[2], 3, 0, OLED_6X8);
        OLED_ShowFloatNum(90, 48, target_angle[3], 3, 0, OLED_6X8);
      } else {
                float val = target_angle[selected_axis];
                OLED_ShowFloatNum(42, 48, val, 3, 0, OLED_6X8);
            }
        } else {
            if (selected_axis == 0) {
                OLED_ShowFloatNum(42, 48, target_pos[0], 3, 0, OLED_6X8);
                OLED_ShowFloatNum(66, 48, target_pos[1], 3, 0, OLED_6X8);
                OLED_ShowFloatNum(90, 48, target_pos[2], 3, 0, OLED_6X8);
            } else {
                float val = target_pos[selected_axis - 1];
                OLED_ShowFloatNum(42, 48, val, 3, 0, OLED_6X8);
            }
        }		
				
				if (!target_dead_area && !current_dead_area) {
            OLED_ClearArea(0, 56, 128, 8);
        }
		
		Key_Tick();
		if (Key_Check(KEY_1, KEY_DOWN))
		{
			LED1_ON();
			LED2_OFF();
			LED3_OFF();
			LED4_OFF();
			control_mode = !control_mode;
            selected_axis = 0;
            if (control_mode == 0) {
                target_angle[1] = roundf(angle1 / 10.0f) * 10.0f;
                target_angle[2] = roundf(angle2 / 10.0f) * 10.0f;
                target_angle[3] = roundf(angle3 / 10.0f) * 10.0f;
            } else {
                target_pos[0] = roundf(x / 20.0f) * 20.0f;
                target_pos[1] = roundf(y / 20.0f) * 20.0f;
                target_pos[2] = roundf(z / 20.0f) * 20.0f;
            }
		}
		
		if (Key_Check(KEY_2, KEY_DOWN))
		{
			LED2_ON();
			LED1_OFF();
			LED3_OFF();
			LED4_OFF();
			selected_axis++;
            if (selected_axis > 3) selected_axis = 0;
		}
		
		if (Key_Check(KEY_3, KEY_DOWN))
		{
			LED3_ON();
			LED1_OFF();
			LED2_OFF();
			LED4_OFF();
			if (control_mode == 0) {
                target_angle[selected_axis] += 10.0f;
                if (selected_axis == 2) {
                    if (target_angle[2] > 100.0f) target_angle[2] = 100.0f;
                    if (target_angle[2] < -100.0f) target_angle[2] = -100.0f;
                } else if (selected_axis == 3) {
                    if (target_angle[3] > 165.0f) target_angle[3] = 165.0f;
                    if (target_angle[3] < -165.0f) target_angle[3] = -165.0f;
                } else {
                    if (target_angle[1] > 180.0f) target_angle[1] = 180.0f;
                    if (target_angle[1] < -180.0f) target_angle[1] = -180.0f;
                }
                uint32_t start = HAL_GetTick();
                while (motors[1].request_pending && (HAL_GetTick() - start < 100)) HAL_Delay(1);
                motors[1].request_pending = 0; motors[1].data_ready = 0;
                while (motors[2].request_pending && (HAL_GetTick() - start < 100)) HAL_Delay(1);
                motors[2].request_pending = 0; motors[2].data_ready = 0;
                while (motors[3].request_pending && (HAL_GetTick() - start < 100)) HAL_Delay(1);
                motors[3].request_pending = 0; motors[3].data_ready = 0;

                if (arm_control_angle_pos(target_angle[1], target_angle[2], target_angle[3], 100, 1000))
								{
									EndEffector_SetAxis4Horizontal(target_angle[1], target_angle[2], target_angle[3], 500);
								}
								else {
									target_dead_area = 1;
									OLED_ClearArea(0, 56, 128, 8);
									OLED_ShowString(0, 56, "Error: Target DA", OLED_6X8);
									OLED_Update();
									HAL_Delay(1000);
									target_dead_area = 0;
									OLED_ClearArea(0, 56, 128, 8);
								}
            } else {
                int idx = selected_axis - 1;
                target_pos[idx] += 20.0f;
                if (target_pos[0] > 400.0f) target_pos[0] = 400.0f;
                if (target_pos[0] < -400.0f) target_pos[0] = -400.0f;
                if (target_pos[1] > 400.0f) target_pos[1] = 400.0f;
                if (target_pos[1] < -400.0f) target_pos[1] = -400.0f;
                if (target_pos[2] > 300.0f) target_pos[2] = 300.0f;
                if (target_pos[2] < -210.0f) target_pos[2] = -210.0f;

                uint32_t start = HAL_GetTick();
                while (motors[1].request_pending && (HAL_GetTick() - start < 100)) HAL_Delay(1);
                motors[1].request_pending = 0; motors[1].data_ready = 0;
                while (motors[2].request_pending && (HAL_GetTick() - start < 100)) HAL_Delay(1);
                motors[2].request_pending = 0; motors[2].data_ready = 0;
                while (motors[3].request_pending && (HAL_GetTick() - start < 100)) HAL_Delay(1);
                motors[3].request_pending = 0; motors[3].data_ready = 0;

                if (arm_control_3d_pos(target_pos[0], target_pos[1], target_pos[2], 100, 1000))
								{
									float a1, a2, a3;
									if (get_arm_target_angles(target_pos[0], target_pos[1], target_pos[2], &a1, &a2, &a3))
									{
										EndEffector_SetAxis4Horizontal(a1, a2, a3, 500);
									}
								}
								else {
									target_dead_area = 1;
									OLED_ClearArea(0, 56, 128, 8);
									OLED_ShowString(0, 56, "Error: Target DA", OLED_6X8);
									OLED_Update();
									HAL_Delay(1000);
									target_dead_area = 0;
									OLED_ClearArea(0, 56, 128, 8);
								}
            }
		}
		
		if (Key_Check(KEY_4, KEY_DOWN))
		{
			LED4_ON();
			LED1_OFF();
			LED2_OFF();
			LED3_OFF();
			if (control_mode == 0) {
                target_angle[selected_axis] -= 10.0f;

                if (selected_axis == 2) {
                    if (target_angle[2] > 100.0f) target_angle[2] = 100.0f;
                    if (target_angle[2] < -100.0f) target_angle[2] = -100.0f;
                } else if (selected_axis == 3) {
                    if (target_angle[3] > 165.0f) target_angle[3] = 165.0f;
                    if (target_angle[3] < -165.0f) target_angle[3] = -165.0f;
                } else {
                    if (target_angle[1] > 180.0f) target_angle[1] = 180.0f;
                    if (target_angle[1] < -180.0f) target_angle[1] = -180.0f;
                }

                uint32_t start = HAL_GetTick();
                while (motors[1].request_pending && (HAL_GetTick() - start < 100)) HAL_Delay(1);
                motors[1].request_pending = 0; motors[1].data_ready = 0;
                while (motors[2].request_pending && (HAL_GetTick() - start < 100)) HAL_Delay(1);
                motors[2].request_pending = 0; motors[2].data_ready = 0;
                while (motors[3].request_pending && (HAL_GetTick() - start < 100)) HAL_Delay(1);
                motors[3].request_pending = 0; motors[3].data_ready = 0;

                if (arm_control_angle_pos(target_angle[1], target_angle[2], target_angle[3], 100, 1000))
								{
									EndEffector_SetAxis4Horizontal(target_angle[1], target_angle[2], target_angle[3], 500);
								}
								else {
									target_dead_area = 1;
									OLED_ClearArea(0, 56, 128, 8);
									OLED_ShowString(0, 56, "Error: Target DA", OLED_6X8);
									OLED_Update();
									HAL_Delay(1000);
									target_dead_area = 0;
									OLED_ClearArea(0, 56, 128, 8);
								}
            } else {
                int idx = selected_axis - 1;
                target_pos[idx] -= 20.0f;
                if (target_pos[0] > 400.0f) target_pos[0] = 400.0f;
                if (target_pos[0] < -400.0f) target_pos[0] = -400.0f;
                if (target_pos[1] > 400.0f) target_pos[1] = 400.0f;
                if (target_pos[1] < -400.0f) target_pos[1] = -400.0f;
                if (target_pos[2] > 300.0f) target_pos[2] = 300.0f;
                if (target_pos[2] < -210.0f) target_pos[2] = -210.0f;

                uint32_t start = HAL_GetTick();
                while (motors[1].request_pending && (HAL_GetTick() - start < 100)) HAL_Delay(1);
                motors[1].request_pending = 0; motors[1].data_ready = 0;
                while (motors[2].request_pending && (HAL_GetTick() - start < 100)) HAL_Delay(1);
                motors[2].request_pending = 0; motors[2].data_ready = 0;
                while (motors[3].request_pending && (HAL_GetTick() - start < 100)) HAL_Delay(1);
                motors[3].request_pending = 0; motors[3].data_ready = 0;

                if (arm_control_3d_pos(target_pos[0], target_pos[1], target_pos[2], 100, 1000))
								{
									float a1, a2, a3;
									if (get_arm_target_angles(target_pos[0], target_pos[1], target_pos[2], &a1, &a2, &a3))
									{
										EndEffector_SetAxis4Horizontal(a1, a2, a3, 500);
									}
								}
								else {
									target_dead_area = 1;
									OLED_ClearArea(0, 56, 128, 8);
									OLED_ShowString(0, 56, "Error: Target DA", OLED_6X8);
									OLED_Update();
									HAL_Delay(1000);
									target_dead_area = 0;
									OLED_ClearArea(0, 56, 128, 8);
								}
            }
		}
		
		OLED_Update();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
