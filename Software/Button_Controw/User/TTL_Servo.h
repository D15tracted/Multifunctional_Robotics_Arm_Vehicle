/**
 * @file    bus_servo.h
 * @brief   Driver for ZL-robot serial bus servos (UART5)
 * @date    2026-04-17
 */

#ifndef TTL_SERVO_H
#define TTL_SERVO_H

#include "main.h"   // for HAL and stdint

/* Servo command macros */
#define SERVO_BROADCAST_ID   255
#define SERVO_DEFAULT_ID     0

/* Position range (500 ~ 2500) */
#define SERVO_POS_MIN        500
#define SERVO_POS_MAX        2500
#define SERVO_POS_MID        1500

/* Time range (0 ~ 9999 ms) */
#define SERVO_TIME_MIN       0
#define SERVO_TIME_MAX       9999

/* Function prototypes */
void TTL_Servo_Init(void);                                  // Initialise UART5
void TTL_Servo_SendCmd(uint8_t id, uint16_t pos, uint16_t time_ms); // #IDPposTtime!
void TTL_Servo_ReadVersion(uint8_t id);                     // #IDPVER!
void TTL_Servo_ReadID(uint8_t id);                          // #IDPID!
void TTL_Servo_SetID(uint8_t old_id, uint8_t new_id);       // #old_idPIDnew_id!
void TTL_Servo_ReleaseTorque(uint8_t id);                   // #IDPULK!
void TTL_Servo_RecoverTorque(uint8_t id);                   // #IDPULR!
void TTL_Servo_ReadPosition(uint8_t id);                    // #IDPRAD!
void TTL_Servo_SetMode(uint8_t id, uint8_t mode);           // #IDPMODmode! (1..8)
void TTL_Servo_ReadMode(uint8_t id);                        // #IDPMOD!
void TTL_Servo_ReadVoltageTemp(uint8_t id);                 // #IDPRTE!
void TTL_Servo_FactoryReset(uint8_t id);                    // #IDPCLE!
void TTL_Servo_Stop(uint8_t id);                            // #IDPDST!
void TTL_Servo_Pause(uint8_t id);                           // #IDPDPT!
void TTL_Servo_Resume(uint8_t id);                          // #IDPDCT!
uint16_t TTL_Servo_ParsePosition(char *response);

/* Get received response (non-blocking) */
uint8_t* TTL_Servo_GetResponse(void);
void TTL_Servo_ClearResponse(void);

#endif /* TTL_SERVO_H */
