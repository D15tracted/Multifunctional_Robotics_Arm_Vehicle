/**
 * @file    bus_servo.c
 * @brief   Implementation of ZL-robot bus servo driver (UART5)
 * @note    Uses HAL UART5 with interrupt reception
 */

#include "TTL_Servo.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* UART5 handle (declared in main.c) */
extern UART_HandleTypeDef huart5;

/* Receive buffer */
#define RX_BUF_SIZE     64
static uint8_t rx_buffer[RX_BUF_SIZE];
static uint8_t rx_index = 0;
static uint8_t response_ready = 0;
static char last_response[RX_BUF_SIZE] = {0};

static uint8_t rx_byte = 0;

/* Forward declaration of internal function */
static void TTL_Servo_SendString(const char *str);

/**
 * @brief Initialise bus servo driver (enable UART5 RX interrupt)
 */
void TTL_Servo_Init(void)
{
    response_ready = 0;
    rx_index = 0;
    memset(rx_buffer, 0, RX_BUF_SIZE);
    memset(last_response, 0, RX_BUF_SIZE);
    // Start receiving one byte
    HAL_UART_Receive_IT(&huart5, &rx_byte, 1);
}

/**
 * @brief Send a complete command string over UART5
 * @param str Null-terminated command string (e.g. "#000P1500T1000!")
 */
static void TTL_Servo_SendString(const char *str)
{
    HAL_UART_Transmit(&huart5, (uint8_t*)str, strlen(str), 100);
}

/**
 * @brief Generate and send angle control command: #IDPposTtime!
 * @param id       Servo ID (0..254, 255 = broadcast)
 * @param pos      Position (500..2500)
 * @param time_ms  Movement time (0..9999 ms)
 */
void TTL_Servo_SendCmd(uint8_t id, uint16_t pos, uint16_t time_ms)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "#%03dP%04dT%04d!", id, pos, time_ms);
    TTL_Servo_SendString(cmd);
}

/**
 * @brief Read firmware version of a servo
 * @param id Servo ID
 */
void TTL_Servo_ReadVersion(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03dPVER!", id);
    TTL_Servo_SendString(cmd);
}

/**
 * @brief Read current ID of a servo (only one servo should be on the bus)
 * @param id The expected ID (e.g. 0)
 */
void TTL_Servo_ReadID(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03dPID!", id);
    TTL_Servo_SendString(cmd);
}

/**
 * @brief Change a servo's ID
 * @param old_id Current ID
 * @param new_id New ID (0..254)
 */
void TTL_Servo_SetID(uint8_t old_id, uint8_t new_id)
{
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "#%03dPID%03d!", old_id, new_id);
    TTL_Servo_SendString(cmd);
}

/**
 * @brief Release torque (servo becomes freely movable)
 * @param id Servo ID
 */
void TTL_Servo_ReleaseTorque(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03dPULK!", id);
    TTL_Servo_SendString(cmd);
}

/**
 * @brief Recover torque (hold position)
 * @param id Servo ID
 */
void TTL_Servo_RecoverTorque(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03dPULR!", id);
    TTL_Servo_SendString(cmd);
}

/**
 * @brief Read current angle position
 * @param id Servo ID
 */
void TTL_Servo_ReadPosition(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03dPRAD!", id);
    TTL_Servo_SendString(cmd);
}

/**
 * @brief Set working mode
 * @param id   Servo ID
 * @param mode 1..8 (see manual: 1=270° CW, 2=270° CCW, 3=180° CW, 4=180° CCW,
 *            5=360° fixed-turn CW, 6=360° fixed-turn CCW,
 *            7=360° timed CW, 8=360° timed CCW)
 */
void TTL_Servo_SetMode(uint8_t id, uint8_t mode)
{
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "#%03dPMOD%d!", id, mode);
    TTL_Servo_SendString(cmd);
}

/**
 * @brief Read current working mode
 * @param id Servo ID
 */
void TTL_Servo_ReadMode(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03dPMOD!", id);
    TTL_Servo_SendString(cmd);
}

/**
 * @brief Read voltage and temperature (returns like "#000T1927-08.1!")
 * @param id Servo ID
 */
void TTL_Servo_ReadVoltageTemp(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03dPRTE!", id);
    TTL_Servo_SendString(cmd);
}

/**
 * @brief Full factory reset (ID becomes 0, all parameters default)
 * @param id Servo ID
 */
void TTL_Servo_FactoryReset(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03dPCLE!", id);
    TTL_Servo_SendString(cmd);
}

/**
 * @brief Stop motion immediately (cannot resume)
 * @param id Servo ID
 */
void TTL_Servo_Stop(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03dPDST!", id);
    TTL_Servo_SendString(cmd);
}

/**
 * @brief Pause motion (can be resumed with Resume)
 * @param id Servo ID
 */
void TTL_Servo_Pause(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03dPDPT!", id);
    TTL_Servo_SendString(cmd);
}

/**
 * @brief Resume a paused motion
 * @param id Servo ID
 */
void TTL_Servo_Resume(uint8_t id)
{
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03dPDCT!", id);
    TTL_Servo_SendString(cmd);
}

uint16_t TTL_Servo_ParsePosition(char *response)
{
    char *p = strchr(response, 'P');
    if (!p) return 0;
    p++;
    int val = atoi(p);
    if (val < 500) val = 500;
    if (val > 2500) val = 2500;
    return (uint16_t)val;
}

/**
 * @brief Get the last received response string
 * @return Pointer to static buffer (empty if no new response)
 */
uint8_t* TTL_Servo_GetResponse(void)
{
    if (response_ready)
    {
        response_ready = 0;
        return (uint8_t*)last_response;
    }
    return NULL;
}

/**
 * @brief Clear the response buffer manually
 */
void TTL_Servo_ClearResponse(void)
{
    response_ready = 0;
    memset(last_response, 0, sizeof(last_response));
    rx_index = 0;
}

/* UART5 interrupt callback (add to stm32f4xx_it.c) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        // Store received byte
        if (rx_index < RX_BUF_SIZE - 1)
        {
            rx_buffer[rx_index++] = rx_byte;
            if (rx_byte == '!')
            {
                // Full response received
                memcpy(last_response, rx_buffer, rx_index);
                last_response[rx_index] = '\0';
                response_ready = 1;
                rx_index = 0;
            }
        }
        else
        {
            // Buffer overflow, reset
            rx_index = 0;
        }
        // Start next reception
        HAL_UART_Receive_IT(&huart5, &rx_byte, 1);
    }
}
