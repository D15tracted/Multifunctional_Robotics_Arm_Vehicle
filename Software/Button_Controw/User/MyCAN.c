#include "stm32f4xx_hal.h"
#include "can.h" 
#include "MyCAN.h"

/* Global variables */
volatile uint8_t can_rx_new_data = 0;   /**< New data flag */
uint8_t can_rx_last_data[8] = {0};      /**< Last received data */
uint8_t can_rx_last_len = 0;            /**< Length of last data */

can_frame_t g_can_frame = {0};          /**< Frame reassembly buffer */

/**
 * @brief Initialize CAN filters and start CAN peripheral
 * @note  Assumes basic CAN parameters are already configured by MX_CAN_Init()
 */
void MyCAN_Init(void)
{
    CAN_FilterTypeDef can_filter = {0};

    /* Configure filter to accept all IDs (mask mode) */
    can_filter.FilterBank = 0;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start CAN peripheral */
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Enable RX FIFO0 message pending interrupt */
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief Transmit a CAN message (blocking with timeout)
 * @param ID      Extended CAN identifier (29-bit)
 * @param Length  Data length (0-8)
 * @param Data    Pointer to data bytes
 */
void MyCAN_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint32_t timeout = 0;

    txHeader.StdId = 0;
    txHeader.ExtId = ID;
    txHeader.IDE = CAN_ID_EXT;          /* Use extended ID */
    txHeader.RTR = CAN_RTR_DATA;        /* Data frame */
    txHeader.DLC = Length;
    txHeader.TransmitGlobalTime = DISABLE;

    /* Add message to TX mailbox */
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, Data, &txMailbox) != HAL_OK)
    {
        return;
    }

    /* Wait for transmission complete (polling) */
    switch (txMailbox)
    {
        case 0:
            while (!(hcan1.Instance->TSR & CAN_TSR_RQCP0))
            {
                timeout++;
                if (timeout > 1000) break;
            }
            hcan1.Instance->TSR |= CAN_TSR_RQCP0;
            break;
        case 1:
            while (!(hcan1.Instance->TSR & CAN_TSR_RQCP1))
            {
                timeout++;
                if (timeout > 1000) break;
            }
            hcan1.Instance->TSR |= CAN_TSR_RQCP1;
            break;
        case 2:
            while (!(hcan1.Instance->TSR & CAN_TSR_RQCP2))
            {
                timeout++;
                if (timeout > 1000) break;
            }
            hcan1.Instance->TSR |= CAN_TSR_RQCP2;
            break;
        default:
            break;
    }
}

/**
 * @brief Transmit a long message by splitting into multiple 8-byte frames
 * @param ID      CAN identifier
 * @param data    Pointer to data buffer
 * @param len     Total length
 */
void MyCAN_TransmitLong(uint32_t ID, uint8_t *data, uint16_t len)
{
    uint16_t offset = 0;
    while (offset < len) {
        uint8_t send_len = (len - offset > 8) ? 8 : (len - offset);
        MyCAN_Transmit(ID, send_len, data + offset);
        offset += send_len;
    }
}

/**
 * @brief Check if any CAN message is pending in FIFO0
 * @return 1 if data available, 0 otherwise
 */
uint8_t MyCAN_ReceiveFlag(void)
{
    uint32_t fifo_level = (hcan1.Instance->RF0R & CAN_RF0R_FMP0) >> CAN_RF0R_FMP0_Pos;
    return (fifo_level > 0) ? 1 : 0;
}

/**
 * @brief Receive a CAN message from FIFO0 (non-blocking)
 * @param ID      Returned identifier (extended ID)
 * @param Length  Returned data length
 * @param Data    Buffer to store data (at least 8 bytes)
 */
void MyCAN_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data)
{
    CAN_RxHeaderTypeDef rxHeader;

    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, Data) != HAL_OK)
    {
        *ID = 0;
        *Length = 0;
        return;
    }

    if (rxHeader.IDE == CAN_ID_STD)
        *ID = rxHeader.StdId;
    else
        *ID = rxHeader.ExtId;

    *Length = (rxHeader.RTR == CAN_RTR_DATA) ? rxHeader.DLC : 0;
}
