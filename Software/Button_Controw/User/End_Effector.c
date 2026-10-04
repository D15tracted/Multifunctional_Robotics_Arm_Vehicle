/**
 * @file    End_Effector.c
 * @brief   Implementation of end effector control using ZL-robot bus servos (UART5)
 * @note    Servo commands are queued and transmitted by TIM7 in parallel with
 *          motor commands (CAN1). Position is read asynchronously via TIM6.
 */

#include "End_Effector.h"
#include "Stepper_Arm.h"
#include "TTL_Servo.h"
#include "tim.h"               /* for htim6, htim7 */
#include "stm32f4xx_hal.h"
#include <math.h>
#include <string.h>

/* ---------- Servo command queue ---------- */
servo_cmd_t servo_queue[SERVO_CMD_QUEUE_SIZE];
uint8_t     servo_queue_head  = 0;
uint8_t     servo_queue_tail  = 0;
uint8_t     servo_queue_count = 0;

/* ---------- Servo states ---------- */
servo_read_state_t servo_states[7] = {0};

/* ---------- Angle <-> Position helpers ---------- */
static float ServoPosToAngle(uint16_t pos)
{
    if (pos < SERVO_POS_MIN) pos = SERVO_POS_MIN;
    if (pos > SERVO_POS_MAX) pos = SERVO_POS_MAX;
    return ((float)(pos - 500) * 270.0f / 2000.0f) - 135.0f;
}

static uint16_t AngleToServoPos(float angle_deg)
{
    float clamped = angle_deg;
    if (clamped < AXIS_ANGLE_MIN) clamped = AXIS_ANGLE_MIN;
    if (clamped > AXIS_ANGLE_MAX) clamped = AXIS_ANGLE_MAX;
    float servo_angle = clamped + 135.0f;
    uint16_t pos = (uint16_t)(500.0f + (servo_angle * 2000.0f / 270.0f));
    if (pos < SERVO_POS_MIN) pos = SERVO_POS_MIN;
    if (pos > SERVO_POS_MAX) pos = SERVO_POS_MAX;
    return pos;
}

static inline float deg_to_rad(float deg) {
    return deg * (float)(M_PI / 180.0);
}

/* ---- kinematics helpers unchanged (compute_wrist_position, rotate_around_axis) ---- */
static void compute_wrist_position(float a1, float a2z, float a3, float *x, float *y, float *z) {
    float t1 = deg_to_rad(a1);
    float t2 = deg_to_rad(90.0f - a2z);
    float t3 = deg_to_rad(a3);
    float horiz = ARM_LINK_LENGTH * cosf(t2) + ARM_LINK_LENGTH * cosf(t2 + t3);
    *x = horiz * cosf(t1);
    *y = horiz * sinf(t1);
    *z = ARM_LINK_LENGTH * sinf(t2) + ARM_LINK_LENGTH * sinf(t2 + t3);
}

static void rotate_around_axis(const float v[3], const float u[3], float a, float result[3])
{
    float ca = cosf(a), sa = sinf(a);
    float cx = u[1]*v[2] - u[2]*v[1];
    float cy = u[2]*v[0] - u[0]*v[2];
    float cz = u[0]*v[1] - u[1]*v[0];
    float dot = u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
    result[0] = v[0]*ca + cx*sa + u[0]*dot*(1.0f-ca);
    result[1] = v[1]*ca + cy*sa + u[1]*dot*(1.0f-ca);
    result[2] = v[2]*ca + cz*sa + u[2]*dot*(1.0f-ca);
}

/* ---------- Async position read ---------- */
void EndEffector_RequestServoPosition(uint8_t id) {
    if (id < 4 || id > 6) return;
    if (servo_states[id].request_pending) return;
    servo_states[id].request_pending = 1;
    servo_states[id].data_ready = 0;
    TTL_Servo_ReadPosition(id);
}

void EndEffector_OnServoPositionReceived(uint8_t id, uint16_t pos) {
    if (id < 4 || id > 6) return;
    if (pos < SERVO_POS_MIN) pos = SERVO_POS_MIN;
    if (pos > SERVO_POS_MAX) pos = SERVO_POS_MAX;
    servo_states[id].raw_pos = pos;
    servo_states[id].angle   = ServoPosToAngle(pos);
    servo_states[id].request_pending = 0;
    servo_states[id].data_ready = 1;
}

/* ---------- Queue management ---------- */
void EndEffector_EnqueueServoCmd(uint8_t id, uint16_t pos, uint16_t time_ms) {
    if (servo_queue_count >= SERVO_CMD_QUEUE_SIZE) return;
    uint8_t next = (servo_queue_tail + 1) % SERVO_CMD_QUEUE_SIZE;
    servo_queue[servo_queue_tail].id      = id;
    servo_queue[servo_queue_tail].pos     = pos;
    servo_queue[servo_queue_tail].time_ms = time_ms;
    servo_queue_tail = next;
    servo_queue_count++;
}

void EndEffector_StartServoQueue(void) {
    if ((htim7.Instance->CR1 & TIM_CR1_CEN) == 0) {
        HAL_TIM_Base_Stop_IT(&htim6);
        __HAL_TIM_SET_COUNTER(&htim7, 0);
        HAL_TIM_Base_Start_IT(&htim7);
    }
}

void EndEffector_ProcessServoQueue(void) {
    if (servo_queue_count == 0) return;
    servo_cmd_t *cmd = &servo_queue[servo_queue_head];
    TTL_Servo_SendCmd(cmd->id, cmd->pos, cmd->time_ms);
    servo_queue_head = (servo_queue_head + 1) % SERVO_CMD_QUEUE_SIZE;
    servo_queue_count--;
}

/* ---------- Init ---------- */
void EndEffector_Init(void) {
    TTL_Servo_ClearResponse();
    memset(servo_states, 0, sizeof(servo_states));
    servo_queue_head = servo_queue_tail = servo_queue_count = 0;
}

/* ---------- Blocking reads (fallback) ---------- */
static uint16_t BlockingReadPos(uint8_t id) {
    TTL_Servo_ClearResponse();
    TTL_Servo_ReadPosition(id);
    uint32_t timeout = 50;
    while (timeout--) {
        uint8_t *resp = TTL_Servo_GetResponse();
        if (resp) return TTL_Servo_ParsePosition((char *)resp);
        HAL_Delay(1);
    }
    return SERVO_MID_POS;
}

void EndEffector_ReadAxis4Angle(float *angle) {
    *angle = ServoPosToAngle(BlockingReadPos(AXIS4_SERVO_ID));
}
void EndEffector_ReadAxis5Angle(float *angle) {
    *angle = ServoPosToAngle(BlockingReadPos(AXIS5_SERVO_ID));
}

/* ---------- Command functions (enqueue instead of direct send) ---------- */
void EndEffector_SetAxis4Angle(float angle_deg, uint16_t time_ms) {
    //angle_deg = -angle_deg;   // invert direction
    uint16_t pos = AngleToServoPos(angle_deg);
    EndEffector_EnqueueServoCmd(AXIS4_SERVO_ID, pos, time_ms);
    EndEffector_StartServoQueue();
}

void EndEffector_SetAxis5Angle(float angle_deg, uint16_t time_ms) {
    uint16_t pos = AngleToServoPos(angle_deg);
    EndEffector_EnqueueServoCmd(AXIS5_SERVO_ID, pos, time_ms);
    EndEffector_StartServoQueue();
}

void EndEffector_Gripper_Open(uint16_t time_ms) {
    EndEffector_EnqueueServoCmd(GRIPPER_SERVO_ID, GRIPPER_OPEN_POS, time_ms);
    EndEffector_StartServoQueue();
}

void EndEffector_Gripper_Close(uint16_t time_ms) {
    EndEffector_EnqueueServoCmd(GRIPPER_SERVO_ID, GRIPPER_CLOSE_POS, time_ms);
    EndEffector_StartServoQueue();
}

void EndEffector_SetGripper(uint16_t pos, uint16_t time_ms) {
    if (pos < SERVO_POS_MIN) pos = SERVO_POS_MIN;
    if (pos > SERVO_POS_MAX) pos = SERVO_POS_MAX;
    EndEffector_EnqueueServoCmd(GRIPPER_SERVO_ID, pos, time_ms);
    EndEffector_StartServoQueue();
}

/* High-level posture */
void EndEffector_SetAxis4Vertical(float angle1_deg, float angle2_z_deg,
                                  float angle3_deg, uint16_t time_ms) {
    float theta2_horiz_deg = 90.0f - angle2_z_deg;
    float theta_sum_deg    = theta2_horiz_deg + angle3_deg;
    float theta_sum_rad    = deg_to_rad(theta_sum_deg);
    float phi_rad = (float)(M_PI / 2.0) + theta_sum_rad;
    while (phi_rad >  (float)M_PI)  phi_rad -= (float)(2.0 * M_PI);
    while (phi_rad < -(float)M_PI)  phi_rad += (float)(2.0 * M_PI);
    float phi_deg = phi_rad * (float)(180.0 / M_PI);
    if      (phi_deg >  90.0f) phi_deg =  90.0f;
    else if (phi_deg < -90.0f) phi_deg = -90.0f;
    EndEffector_SetAxis4Angle(phi_deg, time_ms);
    (void)angle1_deg;
}

void EndEffector_SetAxis4Horizontal(float angle1_deg, float angle2_z_deg,
                                    float angle3_deg, uint16_t time_ms) {
    float theta2_horiz_deg = 90.0f - angle2_z_deg;
    float theta_sum_deg    = theta2_horiz_deg + angle3_deg;
    float theta_sum_rad    = deg_to_rad(theta_sum_deg);
    float phi_rad = -theta_sum_rad;
    while (phi_rad >  (float)M_PI)  phi_rad -= (float)(2.0 * M_PI);
    while (phi_rad < -(float)M_PI)  phi_rad += (float)(2.0 * M_PI);
    float phi_deg = phi_rad * (float)(180.0 / M_PI);
    if      (phi_deg >  90.0f) phi_deg =  90.0f;
    else if (phi_deg < -90.0f) phi_deg = -90.0f;
    EndEffector_SetAxis4Angle(phi_deg, time_ms);
    (void)angle1_deg;
}

/* -------------------------------------------------------------------------- */
/*  Collision check functions                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Check if the two critical end-effector points enter the forbidden
 *         zone when the gripper is held VERTICAL.
 *
 * @param  angle1_deg       axis1 angle (with X-axis)
 * @param  angle2_z_deg     axis2 angle (with Z-axis, -90 = horizontal)
 * @param  angle3_deg       axis3 angle (relative to extension of first link)
 * @param  axis5_angle_deg  current/target angle of axis5 (wrist roll) in degrees
 *
 * @return 1 if both points are safe, 0 if any point lies in dead_area()
 *
 * @note   Point1: wrist + 250 mm straight down (-Z).
 *         Point2: wrist + 100 mm downward (-Z) + 60 mm opposite to axis1 facing
 *         direction (both offsets are then rotated by axis5 angle around the
 *         gripper axis).
 */
int EndEffector_CheckCollisionVertical(float angle1_deg, float angle2_z_deg,
                                       float angle3_deg, float axis5_angle_deg)
{
    /* ---- 1. Wrist position (end of second link) ---- */
    float wrist[3];
    compute_wrist_position(angle1_deg, angle2_z_deg, angle3_deg,
                           &wrist[0], &wrist[1], &wrist[2]);

    /* ---- 2. Axis1 facing direction (in the XY plane) ---- */
    float theta1 = deg_to_rad(angle1_deg);
    float dir_axis1[3] = { cosf(theta1), sinf(theta1), 0.0f };

    /* ---- 3. Point 1: wrist + (0, 0, -250) mm ---- */
    float p1[3] = { wrist[0], wrist[1], wrist[2] - 250.0f };
    if (!dead_area(p1[0], p1[1], p1[2]))
        return 0;

    /* ---- 4. Gripper axis (unit vector) when gripper is vertical ---- */
    /* Second link orientation */
    float theta2_horiz_deg = 90.0f - angle2_z_deg;
    float theta_sum = deg_to_rad(theta2_horiz_deg + angle3_deg);
    float u_link[3] = {
        cosf(theta_sum) * cosf(theta1),
        cosf(theta_sum) * sinf(theta1),
        sinf(theta_sum)
    };

    /* Rotation axis of axis4 (normal to the arm plane) */
    float n[3] = { -sinf(theta1), cosf(theta1), 0.0f };

    /* Ideal axis4 angle for vertical gripper (unclamped first) */
    float phi_ideal = (float)M_PI / 2.0f + theta_sum;
    /* Clamp to physical limits */
    if (phi_ideal >  (float)M_PI / 2.0f) phi_ideal =  (float)M_PI / 2.0f;
    if (phi_ideal < -(float)M_PI / 2.0f) phi_ideal = -(float)M_PI / 2.0f;

    /* Rotate u_link around n by phi_ideal to obtain gripper axis */
    float gripper_axis[3];
    rotate_around_axis(u_link, n, phi_ideal, gripper_axis);
    /* Normalise (should already be unit) */
    float len = sqrtf(gripper_axis[0]*gripper_axis[0] +
                      gripper_axis[1]*gripper_axis[1] +
                      gripper_axis[2]*gripper_axis[2]);
    gripper_axis[0] /= len;
    gripper_axis[1] /= len;
    gripper_axis[2] /= len;

    /* ---- 5. Point 2 base offset (axis5 at 0°) ---- */
    /* For vertical gripper: 100 mm down (-Z) + 60 mm opposite to axis1 facing */
    float d0[3] = {
        -60.0f * dir_axis1[0],
        -60.0f * dir_axis1[1],
        -100.0f
    };

    /* Rotate d0 around gripper_axis by axis5 angle */
    float psi = deg_to_rad(axis5_angle_deg);
    float d_rot[3];
    rotate_around_axis(d0, gripper_axis, psi, d_rot);

    float p2[3] = { wrist[0] + d_rot[0],
                    wrist[1] + d_rot[1],
                    wrist[2] + d_rot[2] };

    if (!dead_area(p2[0], p2[1], p2[2]))
        return 0;

    return 1;
}

/**
 * @brief  Check if the two critical end-effector points enter the forbidden
 *         zone when the gripper is held HORIZONTAL.
 *
 * @param  angle1_deg       axis1 angle (with X-axis)
 * @param  angle2_z_deg     axis2 angle (with Z-axis, -90 = horizontal)
 * @param  angle3_deg       axis3 angle (relative to extension of first link)
 * @param  axis5_angle_deg  current/target angle of axis5 (wrist roll) in degrees
 *
 * @return 1 if both points are safe, 0 if any point lies in dead_area()
 *
 * @note   Point1: wrist + 250 mm towards axis1 facing direction.
 *         Point2: wrist + 100 mm towards axis1 facing direction
 *                      + 60 mm downward (-Z)
 *         (both offsets are then rotated by axis5 angle around the gripper axis).
 */
int EndEffector_CheckCollisionHorizontal(float angle1_deg, float angle2_z_deg,
                                         float angle3_deg, float axis5_angle_deg)
{
    /* ---- 1. Wrist position ---- */
    float wrist[3];
    compute_wrist_position(angle1_deg, angle2_z_deg, angle3_deg,
                           &wrist[0], &wrist[1], &wrist[2]);

    /* ---- 2. Axis1 facing direction ---- */
    float theta1 = deg_to_rad(angle1_deg);
    float dir_axis1[3] = { cosf(theta1), sinf(theta1), 0.0f };

    /* ---- 3. Point 1: wrist + 250 mm along axis1 facing ---- */
    float p1[3] = {
        wrist[0] + 250.0f * dir_axis1[0],
        wrist[1] + 250.0f * dir_axis1[1],
        wrist[2]
    };
    if (!dead_area(p1[0], p1[1], p1[2]))
        return 0;

    /* ---- 4. Gripper axis (unit vector) when gripper is horizontal ---- */
    float theta2_horiz_deg = 90.0f - angle2_z_deg;
    float theta_sum = deg_to_rad(theta2_horiz_deg + angle3_deg);
    float u_link[3] = {
        cosf(theta_sum) * cosf(theta1),
        cosf(theta_sum) * sinf(theta1),
        sinf(theta_sum)
    };
    float n[3] = { -sinf(theta1), cosf(theta1), 0.0f };

    float phi_ideal = -theta_sum;
    if (phi_ideal >  (float)M_PI / 2.0f) phi_ideal =  (float)M_PI / 2.0f;
    if (phi_ideal < -(float)M_PI / 2.0f) phi_ideal = -(float)M_PI / 2.0f;

    float gripper_axis[3];
    rotate_around_axis(u_link, n, phi_ideal, gripper_axis);
    float len = sqrtf(gripper_axis[0]*gripper_axis[0] +
                      gripper_axis[1]*gripper_axis[1] +
                      gripper_axis[2]*gripper_axis[2]);
    gripper_axis[0] /= len;
    gripper_axis[1] /= len;
    gripper_axis[2] /= len;

    /* ---- 5. Point 2 base offset (axis5 at 0°) ---- */
    /* For horizontal gripper: 100 mm towards axis1 facing + 60 mm down (-Z) */
    float d0[3] = {
        100.0f * dir_axis1[0],
        100.0f * dir_axis1[1],
        -60.0f
    };

    float psi = deg_to_rad(axis5_angle_deg);
    float d_rot[3];
    rotate_around_axis(d0, gripper_axis, psi, d_rot);

    float p2[3] = { wrist[0] + d_rot[0],
                    wrist[1] + d_rot[1],
                    wrist[2] + d_rot[2] };

    if (!dead_area(p2[0], p2[1], p2[2]))
        return 0;

    return 1;
}
