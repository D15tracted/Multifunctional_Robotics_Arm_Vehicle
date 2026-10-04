/**
 * @file Stepper_Arm.c
 * @brief 3-DOF robotic arm control library with CAN-based stepper motors
 * @note  Coordinate system: X-Y horizontal plane, Z vertical.
 *        Joint angles:
 *          - Axis1: angle with X-axis, initial 0 (range -180deg to 180deg)
 *          - Axis2: angle between first link and Z-axis, initial -90deg (horizontal) (range -100deg to 100deg)
 *          - Axis3: angle between second link and extension of first link, initial +165deg (range -165deg to 165deg)
 *        Harmonic drive reverses output direction: motor pulses count negative sign.
 */

#include "stm32f4xx_hal.h"
#include "can.h"
#include "tim.h"
#include "MyCAN.h"
#include "smd.h"
#include "OLED.h"
#include <stdint.h>
#include <limits.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define MOTOR_COUNT 3

typedef struct {
    volatile uint8_t request_pending;   
    volatile uint8_t data_ready;        
    int32_t position;                   
    float angle;                        
} motor_read_state_t;

typedef struct {
    uint8_t addr;
    uint8_t dir;
    uint8_t acc;
    uint16_t speed;
    uint32_t pulses;
} motor_cmd_t;

motor_read_state_t motors[4] = {0};   /**< Motor state array (index 1..3 used) */
motor_cmd_t cmd_queue[3];             /**< Command queue for sequential transmission */
uint8_t cmd_queue_count = 0;          /**< Number of commands currently queued */
uint8_t cmd_queue_sent = 0;           /**< Number of commands already sent */

/* Constants */
#define ARM_LINK_LENGTH         245.0f      /* mm, length of each link */
#define MOTOR_PULSES_PER_REV    51200.0f    /* pulses per motor revolution */
#define GEAR_RATIO              100.0f      /* harmonic drive gear ratio */
/* RAD_PER_PULSE includes negative sign for harmonic drive reverse direction */
#define RAD_PER_PULSE           (2.0f * M_PI / (-MOTOR_PULSES_PER_REV * GEAR_RATIO))

/* Default joint angles (degrees) for zero position */
#define DEFAULT_ANGLE1_DEG      0.0f        /* axis1: 0deg (along X-axis) */
#define DEFAULT_ANGLE2_DEG      -90.0f      /* axis2: -90deg (first link horizontal) */
#define DEFAULT_ANGLE3_DEG      -160.0f      /* axis3: 150deg (second link folded) */

/* CAN frame constants */
#define FRAME_HEADER            0xC5
#define FRAME_TAIL              0x5C
#define CMD_READ_POS            0x2A
#define MIN_FRAME_LEN           5
#define POS_DATA_OFFSET         4           /* position bytes start at buf[4] */
#define POS_DATA_LEN            4

#define ANGLE_NO_CHANGE     1000.0f     /* special value meaning "do not change this axis" */

/* Helper: convert degrees to radians */
static inline float deg_to_rad(float deg) {
    return deg * M_PI / 180.0f;
}

/* Helper: convert radians to degrees */
static inline float rad_to_deg(float rad) {
    return rad * 180.0f / M_PI;
}

float pulse_to_angle(uint8_t addr, int32_t pulse) {
    /* Convert raw pulse count to joint angle in degrees */
    float rad_delta = (float)pulse * RAD_PER_PULSE;
    switch (addr) {
        case 1: return 0.0f - rad_to_deg(rad_delta);
        case 2: return -90.0f - rad_to_deg(rad_delta);
        case 3: return -160.0f - rad_to_deg(rad_delta);
        default: return 0.0f;
    }
}

/**
 * @brief Request asynchronous reading of motor position
 * @param addr Motor address (1-3)
 * @note  Sets request_pending flag; result will be stored when CAN response arrives
 */
void read_motor_position(uint8_t addr) {
    if (addr < 1 || addr > 3) return;
    if (motors[addr].request_pending) return;
    
    motors[addr].request_pending = 1;
    motors[addr].data_ready = 0;
    smd_read_pos(addr);
}

/**
 * @brief Add a motor command to the transmission queue
 * @param addr   Motor address
 * @param dir    Direction (0=CW, 1=CCW)
 * @param acc    Acceleration (RPM/s)
 * @param speed  Maximum speed (RPM)
 * @param pulses Number of pulses to move (absolute value)
 */
void enqueue_motor_cmd(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed, uint32_t pulses) {
    if (cmd_queue_count >= 3) return;
    cmd_queue[cmd_queue_count].addr = addr;
    cmd_queue[cmd_queue_count].dir = dir;
    cmd_queue[cmd_queue_count].acc = acc;
    cmd_queue[cmd_queue_count].speed = speed;
    cmd_queue[cmd_queue_count].pulses = pulses;
    cmd_queue_count++;
}

/**
 * @brief Start sending queued commands using TIM7 interrupt
 * @note  Stops TIM6 periodic reading during command transmission
 */
void start_cmd_transmission(void) {
    cmd_queue_sent = 0;
		HAL_TIM_Base_Stop_IT(&htim6);
    __HAL_TIM_SET_COUNTER(&htim7, 0);
    HAL_TIM_Base_Start_IT(&htim7);
}

/**
 * @brief Set current joint angles as zero reference (send angle-zero command to each motor)
 */
void set_angle_zero(void)
{
		HAL_TIM_Base_Stop_IT(&htim6);
		HAL_Delay(100);
    smd_angle_to_zero(1);
		HAL_Delay(100);
    smd_angle_to_zero(2);
		HAL_Delay(100);
    smd_angle_to_zero(3);
		HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief Set arm enable
 */
void arm_enable(void)
{
		HAL_TIM_Base_Stop_IT(&htim6);
		HAL_Delay(100);
    smd_motor_enable(1, 0);
		HAL_Delay(100);
    smd_motor_enable(2, 0);
		HAL_Delay(100);
    smd_motor_enable(3, 0);
		HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief Set arm disable
 */
void arm_disable(void)
{
		HAL_TIM_Base_Stop_IT(&htim6);
		HAL_Delay(100);
    smd_motor_enable(1, 1);
		HAL_Delay(100);
    smd_motor_enable(2, 1);
		HAL_Delay(100);
    smd_motor_enable(3, 1);
		HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief Forward kinematics: compute end-effector coordinates from joint angles
 * @param angle1_deg   Axis1 angle (with X-axis)
 * @param angle2_z_deg Axis2 angle (with Z-axis, -90deg = horizontal)
 * @param angle3_deg   Axis3 angle (between second link and extension of first link)
 * @param x,y,z        Output coordinates (mm)
 */
static void forward_kinematics(float angle1_deg, float angle2_z_deg, float angle3_deg, float *x, float *y, float *z)
{
    float theta1 = deg_to_rad(angle1_deg);
    float theta2 = deg_to_rad(90.0f - angle2_z_deg);   /* convert to horizontal angle */
    float theta3 = deg_to_rad(angle3_deg);

    float horiz = ARM_LINK_LENGTH * cosf(theta2) + ARM_LINK_LENGTH * cosf(theta2 + theta3);
    *x = horiz * cosf(theta1);
    *y = horiz * sinf(theta1);
    *z = ARM_LINK_LENGTH * sinf(theta2) + ARM_LINK_LENGTH * sinf(theta2 + theta3);
}

/**
 * @brief Inverse kinematics for 2-link planar arm (elbow-up configuration)
 * @param x,y,z   Target coordinates (y used only for horizontal angle)
 * @param theta1  Output: axis1 angle (radians)
 * @param theta2  Output: first link horizontal angle (radians)
 * @param theta3  Output: second link relative angle (radians, negative)
 * @return 1 if solution exists, 0 otherwise
 */
static int inverse_kinematics(float x, float y, float z, float *theta1, float *theta2, float *theta3)
{
    float r = sqrtf(x * x + y * y);
    float max_reach = 2.0f * ARM_LINK_LENGTH;
    if (r > max_reach || z > max_reach || (r*r + z*z) > (max_reach*max_reach))
        return 0;

    *theta1 = atan2f(y, x);

    float d = sqrtf(r * r + z * z);
    if (d < 0.001f || d > max_reach)
        return 0;

		//fuck this bug, cause me 8 fucking hours
//    float cos_theta3 = (ARM_LINK_LENGTH*ARM_LINK_LENGTH + ARM_LINK_LENGTH*ARM_LINK_LENGTH - d*d) /
//                       (2.0f * ARM_LINK_LENGTH * ARM_LINK_LENGTH);
    float cos_theta3 = (d * d) / (2.0f * ARM_LINK_LENGTH * ARM_LINK_LENGTH) - 1.0f;
    if (cos_theta3 > 1.0f) cos_theta3 = 1.0f;
    if (cos_theta3 < -1.0f) cos_theta3 = -1.0f;
    *theta3 = -acosf(cos_theta3);   // elbow-up: negative angle

    float alpha = atan2f(z, r);
    float cos_beta = d / (2.0f * ARM_LINK_LENGTH);
    if (cos_beta > 1.0f) cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;
    float beta = acosf(cos_beta);
    *theta2 = alpha + beta;          // elbow-up configuration

    return 1;
}

/**
 * @brief Check if target position is within allowed workspace (dead area)
 * @param x,y,z Target coordinates (mm)
 * @return 1 if safe, 0 if prohibited
 */
int dead_area(float x, float y, float z)
{
    float theta1, theta2, theta3;
    if (!inverse_kinematics(x, y, z, &theta1, &theta2, &theta3))
        return 0;

    float theta1_deg = rad_to_deg(theta1);
    float theta2_deg = rad_to_deg(theta2);
    float theta3_deg = rad_to_deg(theta3);
    float angle_with_z = 90.0f - theta2_deg;   /* axis2 angle (with Z) */

    /* Axis2 angle with Z must be within [-100,100] */
    if (fabsf(angle_with_z) > 100.0f)
        return 0;

    /* Axis3 angle must be within [-162,162]*/
    if (fabsf(theta3_deg) > 162.0f)
        return 0;

    /* No motion below Z = -210 mm */
    if (z < -210.0f)
        return 0;

    /* Rectangular forbidden zone near base */
    if (x >= -285.0f && x <= 60.0f && y >= -150.0f && y <= 150.0f && z < -32.5f)
        return 0;

    /* Additional coupling constraints:
     * - When axis2 (with Z) < -93deg, axis1 must NOT be in range [-90,90]
     * - When axis2 (with Z) > 93deg, axis1 must be in range [-90,90]
		*/
    if (angle_with_z < -93.0f) {
        /* axis1 must be outside [-90,90]*/
        if (theta1_deg >= -90.0f && theta1_deg <= 90.0f)
            return 0;
    }
    if (angle_with_z > 93.0f) {
        /* axis1 must be inside [-90,90]*/
        if (theta1_deg < -90.0f || theta1_deg > 90.0f)
            return 0;
    }

    return 1;
}

/**
 * @brief Check if target joint angles are within allowed workspace
 * @param angle1 Target axis1 angle (with X-axis)
 * @param angle2 Target axis2 angle (with Z-axis)
 * @param angle3 Target axis3 angle (relative)
 * @return 1 if safe, 0 if prohibited
 */
int dead_area_angle(float angle1, float angle2, float angle3)
{
    /* Axis2 range: [-100,100]*/
    if (angle2 < -100.0f || angle2 > 100.0f)
        return 0;

    /* Axis3 range: [-162,162]*/
    if (angle3 < -162.0f || angle3 > 162.0f)
        return 0;

    /* Additional coupling constraints based on axis2 */
    if (angle2 < -93.0f) {
        /* axis1 must be outside [-90,90]*/
        if (angle1 >= -90.0f && angle1 <= 90.0f)
            return 0;
    }
    if (angle2 > 93.0f) {
        /* axis1 must be inside [-90,90]*/
        if (angle1 < -90.0f || angle1 > 90.0f)
            return 0;
    }

    /* Optional: you may also check the Z coordinate via forward kinematics
     * to avoid motion below -210 mm or into the rectangular forbidden zone.
     * But these checks are less relevant when only controlling angles.
     */
    float x, y, z;
    forward_kinematics(angle1, angle2, angle3, &x, &y, &z);
    if (z < -210.0f)
        return 0;
    if (x >= -285.0f && x <= 60.0f && y >= -150.0f && y <= 150.0f && z < -60.0f)
        return 0;

    return 1;
}

/**
 * @brief Control arm to target joint angles (degree interface)
 * @param angle1 Target axis1 angle (with X-axis)
 * @param angle2 Target axis2 angle (with Z-axis)
 * @param angle3 Target axis3 angle (relative)
 * @param acc    Acceleration (0-200 RPM/s, 0 = instant)
 * @param speed  Maximum speed (0-6000 RPM)
 */
uint8_t arm_control_angle_pos(float angle1, float angle2, float angle3, uint8_t acc, uint16_t speed)
{
    /* Check dead area via angle*/
    if (!dead_area_angle(angle1, angle2, angle3))return 0;

    /* Compute required pulse increments from default zero position */
    float delta1_deg = angle1 - DEFAULT_ANGLE1_DEG;
    float delta2_deg = angle2 - DEFAULT_ANGLE2_DEG;
    float delta3_deg = angle3 - DEFAULT_ANGLE3_DEG;

    /* Note: RAD_PER_PULSE already includes harmonic reverse sign */
    int32_t pulse1 = (int32_t)(-deg_to_rad(delta1_deg) / RAD_PER_PULSE);
    int32_t pulse2 = (int32_t)(-deg_to_rad(delta2_deg) / RAD_PER_PULSE);
    int32_t pulse3 = (int32_t)(-deg_to_rad(delta3_deg) / RAD_PER_PULSE);

    /* Helper macro to send position command (inline to reduce duplication) */
    cmd_queue_count = 0;

    enqueue_motor_cmd(1, (pulse1 >= 0) ? 0 : 1, acc, speed, (pulse1 >= 0) ? (uint32_t)pulse1 : (uint32_t)(-pulse1));
    enqueue_motor_cmd(2, (pulse2 >= 0) ? 0 : 1, acc, speed, (pulse2 >= 0) ? (uint32_t)pulse2 : (uint32_t)(-pulse2));
    enqueue_motor_cmd(3, (pulse3 >= 0) ? 0 : 1, acc, speed, (pulse3 >= 0) ? (uint32_t)pulse3 : (uint32_t)(-pulse3));

    start_cmd_transmission();	
		
		return 1;
}

/**
 * @brief Control arm to target Cartesian position (inverse kinematics)
 * @param x,y,z Target coordinates (mm)
 * @param acc   Acceleration (0-200 RPM/s)
 * @param speed Maximum speed (0-6000 RPM)
 */
uint8_t arm_control_3d_pos(float x, float y, float z, uint8_t acc, uint16_t speed)
{
    /* Inverse kinematics */
    float theta1, theta2, theta3;
    if (!inverse_kinematics(x, y, z, &theta1, &theta2, &theta3))
        return 0;

    /* Convert to joint angles in degrees (our definition) */
    float angle1 = rad_to_deg(theta1);
    float angle2 = 90.0f - rad_to_deg(theta2);   /* convert horizontal -> Z-angle */
    float angle3 = rad_to_deg(theta3);

//		OLED_ShowFloatNum(0, 56, angle1, 3, 1, OLED_6X8);
//		OLED_ShowFloatNum(36, 56, angle2, 3, 1, OLED_6X8);
//		OLED_ShowFloatNum(72, 56, angle3, 3, 1, OLED_6X8);
		
    /* Reuse angle control function (includes dead area check) */
    return arm_control_angle_pos(angle1, angle2, angle3, acc, speed);
}

/**
 * @brief Move arm to a safe start position using blocking absolute position commands.
 * @note  Target angles: Axis1=0deg, Axis2=-30deg, Axis3=-120deg
 *        Sends commands sequentially: Axis3 first, then Axis2, finally Axis1.
 *        TIM6 polling is paused during the move to avoid CAN bus contention.
 * @param  acc   Acceleration (0~200 RPM/s, 0 = instant)
 * @param  speed Maximum speed (0~6000 RPM)
 * @return 1 if the target angles are within the allowed workspace, 0 otherwise.
 */
uint8_t arm_control_to_start_pos(uint8_t acc, uint16_t speed)
{
    /* Target angles for the safe start position */
    float target_angle1 = 0.0f;      // Axis1: 0deg
    float target_angle2 = -30.0f;    // Axis2: -30deg(slightly raised from horizontal)
    float target_angle3 = -120.0f;   // Axis3: -120deg(folded, same as default)

    /* Check if the target angles are safe */
    if (!dead_area_angle(target_angle1, target_angle2, target_angle3))
        return 0;

    /* Compute required pulse offsets from the default (zero) position */
    float delta1_deg = target_angle1 - DEFAULT_ANGLE1_DEG;
    float delta2_deg = target_angle2 - DEFAULT_ANGLE2_DEG;
    float delta3_deg = target_angle3 - DEFAULT_ANGLE3_DEG;

    /* RAD_PER_PULSE already includes the harmonic drive reversal */
    int32_t pulse1 = (int32_t)(-deg_to_rad(delta1_deg) / RAD_PER_PULSE);
    int32_t pulse2 = (int32_t)(-deg_to_rad(delta2_deg) / RAD_PER_PULSE);
    int32_t pulse3 = (int32_t)(-deg_to_rad(delta3_deg) / RAD_PER_PULSE);

    uint8_t  dir1 = (pulse1 >= 0) ? 0 : 1;
    uint8_t  dir2 = (pulse2 >= 0) ? 0 : 1;
    uint8_t  dir3 = (pulse3 >= 0) ? 0 : 1;
    uint32_t abs1 = (pulse1 >= 0) ? (uint32_t)pulse1 : (uint32_t)(-pulse1);
    uint32_t abs2 = (pulse2 >= 0) ? (uint32_t)pulse2 : (uint32_t)(-pulse2);
    uint32_t abs3 = (pulse3 >= 0) ? (uint32_t)pulse3 : (uint32_t)(-pulse3);

    /* Pause background position polling to avoid CAN collisions */
    HAL_TIM_Base_Stop_IT(&htim6);

    /* ---- Move Axis3 first (fold the elbow) ---- */
    if (abs3 > 0) {
        smd_pos_mode(3, dir3, acc, speed, abs3);
        /* Wait for the movement to complete (see note below) */
        HAL_Delay(200);
    }

    /* ---- Move Axis2 (lift the arm) ---- */
    if (abs2 > 0) {
        smd_pos_mode(2, dir2, acc, speed, abs2);
        HAL_Delay(200);
    }

    /* ---- Move Axis1 (rotate the base) ---- */
    if (abs1 > 0) {
        smd_pos_mode(1, dir1, acc, speed, abs1);
        HAL_Delay(200);
    }

    /* Resume polling */
    HAL_TIM_Base_Start_IT(&htim6);

    return 1;
}

/**
 * @brief give the 3D position of arm
 * @param x,y,z Target coordinates (mm)
 */
void get_current_position(float *x, float *y, float *z)
{
    forward_kinematics(motors[1].angle, motors[2].angle, motors[3].angle, x, y, z);
}

/**
 * @brief Get target joint angles for a given Cartesian position (inverse kinematics)
 * @param x,y,z Target coordinates (mm)
 * @param angle1,angle2,angle3 Output joint angles in degrees
 * @return 1 if reachable, 0 otherwise
 */
uint8_t get_arm_target_angles(float x, float y, float z, float *angle1, float *angle2, float *angle3)
{
    float theta1, theta2, theta3;
    if (!inverse_kinematics(x, y, z, &theta1, &theta2, &theta3))
        return 0;
    *angle1 = rad_to_deg(theta1);
    *angle2 = 90.0f - rad_to_deg(theta2);   // convert horizontal -> Z-angle
    *angle3 = rad_to_deg(theta3);
    return 1;
}
