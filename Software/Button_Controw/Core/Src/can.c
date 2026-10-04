/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "MyCAN.h"
#include "tim.h"
#include "smd.h"
#include "Stepper_Arm.h"
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
 * @brief Callback when a CAN message is received in FIFO0
 * @param hcan Pointer to CAN handle
 * @note  Reassembles multi-frame messages, verifies checksum,
 *        and updates motor state if the message is a position response.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t rxbuf[8];
    CAN_RxHeaderTypeDef rxHeader;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxbuf);
    
    uint8_t len = rxHeader.DLC;
    memcpy(can_rx_last_data, rxbuf, len);
    can_rx_last_len = len;
    can_rx_new_data = 1;
    
		/* Start TIM8 timeout if this is the first fragment */
    if (g_can_frame.index == 0) {
        __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);
        __HAL_TIM_SET_COUNTER(&htim8, 0);
        __HAL_TIM_ENABLE(&htim8);
    }
		
		/* Append data to reassembly buffer */
    if (g_can_frame.index + len <= CAN_RECV_BUF_LEN) {
        memcpy(&g_can_frame.buf[g_can_frame.index], rxbuf, len);
        g_can_frame.index += len;
        __HAL_TIM_SET_COUNTER(&htim8, 0);	/* Reset timeout */
    } else {
        g_can_frame.index = 0;
        return;
    }
    
		/* Check for complete frame (minimum 5 bytes) */
    if (g_can_frame.index >= 5) {
        if (g_can_frame.buf[0] == FRAME_HEADER &&
            g_can_frame.buf[g_can_frame.index - 1] == FRAME_TAIL) {
						/* Verify checksum (excludes header and tail) */
            uint8_t cal_checksum = smd_checksum(g_can_frame.buf, g_can_frame.index - 2);
            if (cal_checksum == g_can_frame.buf[g_can_frame.index - 2]) {
                uint8_t addr = g_can_frame.buf[1];
                if (addr >= 1 && addr <= 3 && motors[addr].request_pending) {
                    if (g_can_frame.buf[2] == FCT_READ_POS && g_can_frame.index >= 8) {
                        int32_t pos = (int32_t)((g_can_frame.buf[4] << 24) |
                                                (g_can_frame.buf[5] << 16) |
                                                (g_can_frame.buf[6] << 8)  |
                                                (g_can_frame.buf[7]));
                        motors[addr].position = pos;
                        motors[addr].angle = pulse_to_angle(addr, pos);
                    }
                    motors[addr].data_ready = 1;
                    motors[addr].request_pending = 0;
                }
                g_can_frame.frame_done = 1;
								/* Stop timeout timer */
                __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_UPDATE);
                __HAL_TIM_DISABLE(&htim8);
                g_can_frame.index = 0;
            }
        }
    }
}
/* USER CODE END 1 */
