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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LED.h"
#include "OLED.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_VALUES 16
#define BUFFER_SIZE 64

/* PS4 button enumeration (same as before, kept for array indexing) */
enum {
    BTN_R2 = 0,
    BTN_L2,
    BTN_A,
    BTN_B,
    BTN_X,
    BTN_Y,
    BTN_R1,
    BTN_L1,
    BTN_LEFT,
    BTN_UP,
    BTN_RIGHT,
    BTN_DOWN,
    BTN_NUM
};

/* LED pattern mapping for each PS4 button (4-bit: LED4 LED3 LED2 LED1) */
#define LED_PAT_R2      0x01    /* 0001 */
#define LED_PAT_L2      0x02    /* 0010 */
#define LED_PAT_A       0x03    /* 0011 */
#define LED_PAT_B       0x04    /* 0100 */
#define LED_PAT_X       0x05    /* 0101 */
#define LED_PAT_Y       0x06    /* 0110 */
#define LED_PAT_R1      0x07    /* 0111 */
#define LED_PAT_L1      0x08    /* 1000 */
#define LED_PAT_LEFT    0x09    /* 1001 */
#define LED_PAT_UP      0x0A    /* 1010 */
#define LED_PAT_RIGHT   0x0B    /* 1011 */
#define LED_PAT_DOWN    0x0C    /* 1100 */

/* PS4 button names for OLED display */
static const char *ps4_btn_names[BTN_NUM] = {
    "R2", "L2", "A", "B", "X", "Y",
    "R1", "L1", "LEFT", "UP", "RIGHT", "DOWN"
};

static const uint8_t ps4_led_pattern[BTN_NUM] = {
    LED_PAT_R2, LED_PAT_L2, LED_PAT_A, LED_PAT_B,
    LED_PAT_X, LED_PAT_Y, LED_PAT_R1, LED_PAT_L1,
    LED_PAT_LEFT, LED_PAT_UP, LED_PAT_RIGHT, LED_PAT_DOWN
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t led_on_tick = 0;       /* SysTick when LEDs were turned on */
uint8_t  led_active = 0;        /* Flag: LEDs are currently showing a button press */

/* UART receive variables */
uint8_t rx_data;
uint8_t rx_buffer[64];
int rx_index = 0;
int rx_complete = 0;
int value[MAX_VALUES] = {0};

/* Button state tracking (replaces PS4_Arm module) */
static uint8_t btn_prev_state[BTN_NUM];   /* Previous state of each button */
static uint8_t r2_hyst_state = 0;         /* Hysteresis state for R2 */
static uint8_t l2_hyst_state = 0;         /* Hysteresis state for L2 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void SetLEDs(uint8_t pattern);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Set all four LEDs according to a 4-bit pattern.
 * @param  pattern: bit0 -> LED1, bit1 -> LED2, bit2 -> LED3, bit3 -> LED4
 */
static void SetLEDs(uint8_t pattern)
{
    (pattern & 0x01) ? LED1_ON() : LED1_OFF();
    (pattern & 0x02) ? LED2_ON() : LED2_OFF();
    (pattern & 0x04) ? LED3_ON() : LED3_OFF();
    (pattern & 0x08) ? LED4_ON() : LED4_OFF();
}

/**
  * @brief  UART receive complete callback (USART6)
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        if (rx_data == 'c') {
            rx_index = 0;
            rx_complete = 0;
        }
        if (rx_index < 63) {
            rx_buffer[rx_index++] = rx_data;
        }
        if (rx_data == '\n') {
            rx_complete = 1;
        }
        HAL_UART_Receive_IT(&huart6, &rx_data, 1);
    }
}

/**
  * @brief  Parse the received string into the value[] array.
  */
void get_Data(void)
{
    int value_in[MAX_VALUES] = {0};
    int start = 0;
    int times = 0;
    int neg = 0;
    int erro = 0;

    if (rx_complete == 0) {
        return;
    }

    for (int i = 0; i < rx_index; i++) {
        char current = rx_buffer[i];

        if (current == 'c') {
            if (start == 1) {
                erro = 1;
                break;
            }
            start = 1;
            continue;
        }

        if (start == 1) {
            if (current == '\n') {
                break;
            }
            if (current == ',') {
                if (times < MAX_VALUES) {
                    if (neg) value_in[times] = -value_in[times];
                    times++;
                    neg = 0;
                }
            } else if (current == '-') {
                neg = 1;
            } else if (current >= '0' && current <= '9') {
                if (times < MAX_VALUES) {
                    value_in[times] = value_in[times] * 10 + (current - '0');
                }
            }
        }
    }

    if (erro != 1 && times == MAX_VALUES - 1) {
        for (int i = 0; i < MAX_VALUES; i++) {
            value[i] = value_in[i];
        }
    }

    rx_complete = 0;
    rx_index = 0;
}

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
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */
  OLED_Init();
  OLED_ShowString(0, 0, "PS4 Control", OLED_6X8);
  OLED_Update();

  /* Clear button state arrays */
  memset(btn_prev_state, 0, sizeof(btn_prev_state));
  r2_hyst_state = 0;
  l2_hyst_state = 0;

  /* Start UART6 interrupt reception */
  HAL_UART_Receive_IT(&huart6, &rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 1. Receive and parse the latest PS4 data from UART */
    get_Data();

    /* 2. Process each button directly using value[] array */
    for (uint8_t btn = 0; btn < BTN_NUM; btn++) {
        uint8_t curr_state = 0;

        /* Determine current state based on button type */
        if (btn == BTN_R2) {
            if (value[4] > 75) {
                curr_state = 1;
                r2_hyst_state = 1;
            } else if (value[4] < 25) {
                curr_state = 0;
                r2_hyst_state = 0;
            } else {
                curr_state = r2_hyst_state;  /* Hysteresis zone */
            }
        } else if (btn == BTN_L2) {
            if (value[5] > 75) {
                curr_state = 1;
                l2_hyst_state = 1;
            } else if (value[5] < 25) {
                curr_state = 0;
                l2_hyst_state = 0;
            } else {
                curr_state = l2_hyst_state;
            }
        } else {
            /* Digital buttons: map enum to value index */
            uint8_t idx;
            if      (btn == BTN_A)     idx = 6;
            else if (btn == BTN_B)     idx = 7;
            else if (btn == BTN_X)     idx = 8;
            else if (btn == BTN_Y)     idx = 9;
            else if (btn == BTN_R1)    idx = 10;
            else if (btn == BTN_L1)    idx = 11;
            else if (btn == BTN_LEFT)  idx = 12;
            else if (btn == BTN_UP)    idx = 13;
            else if (btn == BTN_RIGHT) idx = 14;
            else if (btn == BTN_DOWN)  idx = 15;
            else continue;  /* Should not happen */

            curr_state = (value[idx] != 0) ? 1 : 0;
        }

        /* Edge detection: rising edge -> button pressed */
        if (curr_state == 1 && btn_prev_state[btn] == 0) {
            /* Set LEDs according to the button's pattern */
            SetLEDs(ps4_led_pattern[btn]);

            /* Display button name on OLED */
            OLED_ShowString(0, 16, "            ", OLED_6X8);   /* Clear line */
            OLED_ShowString(0, 16, "PS4:", OLED_6X8);
            OLED_ShowString(24, 16, (char *)ps4_btn_names[btn], OLED_6X8);
            OLED_Update();

            /* Start 200 ms timer for automatic LED off */
            led_active = 1;
            led_on_tick = HAL_GetTick();

            /* Only one action per loop, break after first detected press */
            btn_prev_state[btn] = curr_state;  /* Update state before breaking */
            break;
        }

        /* Update previous state for next iteration */
        btn_prev_state[btn] = curr_state;
    }

    /* 3. Auto-off LEDs after 200 ms */
    if (led_active && (HAL_GetTick() - led_on_tick >= 200)) {
        SetLEDs(0x00);          /* All LEDs off */
        led_active = 0;         /* Clear active flag */
    }

    /* 4. Optional: display raw values for debugging */
    OLED_Printf(0, 24, OLED_6X8, "%d,%d,%d,%d", value[0], value[1], value[2], value[3]);
    OLED_Printf(0, 32, OLED_6X8, "%d,%d,%d,%d", value[4], value[5], value[6], value[7]);
    OLED_Printf(0, 40, OLED_6X8, "%d,%d,%d,%d", value[8], value[9], value[10], value[11]);
    OLED_Printf(0, 48, OLED_6X8, "%d,%d,%d,%d", value[12], value[13], value[14], value[15]);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
