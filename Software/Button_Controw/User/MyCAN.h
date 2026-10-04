#ifndef __MYCAN_H
#define __MYCAN_H

#define CAN_RECV_BUF_LEN  256   /**< Maximum size of CAN reception buffer */

/** @brief Flag indicating new CAN data has been received (used by main loop) */
extern volatile uint8_t can_rx_new_data;
/** @brief Last received CAN data bytes (up to 8) */
extern uint8_t can_rx_last_data[8];
/** @brief Length of last received CAN data */
extern uint8_t can_rx_last_len;

/**
 * @brief Structure to store a complete CAN frame (fragmented messages reassembled)
 */
typedef struct {
    uint8_t buf[CAN_RECV_BUF_LEN];  /**< Raw data buffer */
    uint16_t index;                 /**< Current fill index */
    volatile uint8_t frame_done;    /**< Flag set when a complete frame is ready */
} can_frame_t;

/** @brief Global CAN frame instance used by reception interrupt */
extern can_frame_t g_can_frame;

/** @brief Initialize CAN hardware and filters */
void MyCAN_Init(void);

/**
 * @brief Transmit a CAN message (standard or extended ID)
 * @param ID      Message identifier (extended ID used)
 * @param Length  Data length (0-8)
 * @param Data    Pointer to data bytes
 */
void MyCAN_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data);

/**
 * @brief Transmit a long message by splitting into multiple CAN frames
 * @param ID      CAN identifier
 * @param data    Pointer to data buffer
 * @param len     Total length (may exceed 8)
 */
void MyCAN_TransmitLong(uint32_t ID, uint8_t *data, uint16_t len);

/**
 * @brief Check if any CAN message is pending in FIFO0
 * @return 1 if data available, 0 otherwise
 */
uint8_t MyCAN_ReceiveFlag(void);

/**
 * @brief Receive a CAN message from FIFO0 (non-blocking)
 * @param ID      Returned message identifier
 * @param Length  Returned data length
 * @param Data    Buffer to store data (at least 8 bytes)
 */
void MyCAN_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data);

#endif /* __MYCAN_H */
