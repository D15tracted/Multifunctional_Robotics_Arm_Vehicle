/**
 * @file    End_Effector.h
 * @brief   End effector control for 2-axis wrist + gripper using ZL-robot bus servos
 * @note    Axis4: wrist pitch, Axis5: wrist roll, Gripper: open/close
 *          Servo commands are queued and sent via TIM7 in parallel with motor commands.
 */

#ifndef END_EFFECTOR_H
#define END_EFFECTOR_H

#include <stdint.h>

/* Servo IDs */
#define AXIS4_SERVO_ID      4
#define AXIS5_SERVO_ID      5
#define GRIPPER_SERVO_ID    6

/* Position limits for 270° bus servos */
#define SERVO_POS_MIN       500
#define SERVO_POS_MAX       2500
#define SERVO_MID_POS       1500

/* Angle limits for wrist axes */
#define AXIS_ANGLE_MIN      (-90.0f)
#define AXIS_ANGLE_MAX      90.0f

/* Gripper pre-defined positions (calibrate as needed) */
#define GRIPPER_OPEN_POS    1850
#define GRIPPER_CLOSE_POS   1250
#define GRIPPER_TIME_MS     500

/* Servo read state (asynchronous) */
typedef struct {
    volatile uint8_t request_pending;
    volatile uint8_t data_ready;
    uint16_t raw_pos;
    float    angle;
} servo_read_state_t;

extern servo_read_state_t servo_states[7];

/* Servo command queue */
#define SERVO_CMD_QUEUE_SIZE  16

typedef struct {
    uint8_t  id;
    uint16_t pos;
    uint16_t time_ms;
} servo_cmd_t;

extern servo_cmd_t servo_queue[SERVO_CMD_QUEUE_SIZE];
extern uint8_t     servo_queue_head;
extern uint8_t     servo_queue_tail;
extern uint8_t     servo_queue_count;

/* Initialisation */
void EndEffector_Init(void);

/* Axis reading */
void EndEffector_ReadAxis4Angle(float *angle);
void EndEffector_ReadAxis5Angle(float *angle);

/* Axis control */
void EndEffector_SetAxis4Angle(float angle_deg, uint16_t time_ms);
void EndEffector_SetAxis5Angle(float angle_deg, uint16_t time_ms);

/* Gripper control */
void EndEffector_Gripper_Open(uint16_t time_ms);
void EndEffector_Gripper_Close(uint16_t time_ms);
void EndEffector_SetGripper(uint16_t pos, uint16_t time_ms);

/* High-level posture */
void EndEffector_SetAxis4Vertical(float angle1_deg, float angle2_z_deg,
                                  float angle3_deg, uint16_t time_ms);
void EndEffector_SetAxis4Horizontal(float angle1_deg, float angle2_z_deg,
                                    float angle3_deg, uint16_t time_ms);

/* Collision checks */
int EndEffector_CheckCollisionVertical(float angle1_deg, float angle2_z_deg,
                                       float angle3_deg, float axis5_angle_deg);
int EndEffector_CheckCollisionHorizontal(float angle1_deg, float angle2_z_deg,
                                         float angle3_deg, float axis5_angle_deg);

/* Asynchronous read functions */
void EndEffector_RequestServoPosition(uint8_t id);
void EndEffector_OnServoPositionReceived(uint8_t id, uint16_t pos);

/* Queue management – called from TIM7 */
void EndEffector_EnqueueServoCmd(uint8_t id, uint16_t pos, uint16_t time_ms);
void EndEffector_StartServoQueue(void);
void EndEffector_ProcessServoQueue(void);

#endif /* END_EFFECTOR_H */
