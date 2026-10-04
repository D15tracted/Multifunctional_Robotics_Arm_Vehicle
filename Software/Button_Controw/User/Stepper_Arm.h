#ifndef __STEPPER_ARM_H
#define __STEPPER_ARM_H

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define MOTOR_COUNT 3   /**< Number of motors in the arm */

/**
 * @brief State structure for asynchronous motor position reading
 */
typedef struct {
    volatile uint8_t request_pending;   /**< Set when a read request is pending */
    volatile uint8_t data_ready;        /**< Set when new data is available */
    int32_t position;                   /**< Raw pulse count */
    float angle;                        /**< Joint angle in degrees */
} motor_read_state_t;

/**
 * @brief Command structure for a single motor position movement
 */
typedef struct {
    uint8_t addr;       /**< Motor address (1-3) */
    uint8_t dir;        /**< Direction: 0 = CW, 1 = CCW */
    uint8_t acc;        /**< Acceleration (RPM/s, 0 = instant) */
    uint16_t speed;     /**< Maximum speed (RPM) */
    uint32_t pulses;    /**< Number of pulses to move */
} motor_cmd_t;

/* Global variables (defined in Stepper_Arm.c) */
extern motor_read_state_t motors[4];
extern motor_cmd_t cmd_queue[3];
extern uint8_t cmd_queue_count;
extern uint8_t cmd_queue_sent;

/* Constants */
#define ARM_LINK_LENGTH         245.0f      /**< Length of each arm link (mm) */
#define MOTOR_PULSES_PER_REV    51200.0f    /**< Encoder pulses per motor revolution */
#define GEAR_RATIO              100.0f      /**< Harmonic drive gear ratio */
#define RAD_PER_PULSE           (2.0f * M_PI / (-MOTOR_PULSES_PER_REV * GEAR_RATIO)) /**< Radians per pulse (includes harmonic reverse) */

/* Default joint angles at zero position (degrees) */
#define DEFAULT_ANGLE1_DEG      0.0f        /**< Axis1: 0deg (along X-axis) */
#define DEFAULT_ANGLE2_DEG      -90.0f      /**< Axis2: -90deg (first link horizontal) */
#define DEFAULT_ANGLE3_DEG      -160.0f      /**< Axis3: 160deg (second link folded) */

/* CAN frame constants */
#define FRAME_HEADER            0xC5
#define FRAME_TAIL              0x5C
#define CMD_READ_POS            0x2A
#define MIN_FRAME_LEN           5
#define POS_DATA_OFFSET         4
#define POS_DATA_LEN            4

#define ANGLE_NO_CHANGE     1000.0f     /**< Special value meaning "do not change this axis" */

/* Helper function prototypes (inline) */
static inline float deg_to_rad(float deg);
static inline float rad_to_deg(float rad);

/* Public functions */
float pulse_to_angle(uint8_t addr, int32_t pulse);
void read_motor_position(uint8_t addr);
void enqueue_motor_cmd(uint8_t addr, uint8_t dir, uint8_t acc, uint16_t speed, uint32_t pulses);
void start_cmd_transmission(void);
void set_angle_zero(void);
void arm_enable(void);
void arm_disable(void);
static void forward_kinematics(float angle1_deg, float angle2_z_deg, float angle3_deg, float *x, float *y, float *z);
static int inverse_kinematics(float x, float y, float z, float *theta1, float *theta2, float *theta3);
int dead_area(float x, float y, float z);
int dead_area_angle(float angle1, float angle2, float angle3);
uint8_t arm_control_angle_pos(float angle1, float angle2, float angle3, uint8_t acc, uint16_t speed);
uint8_t arm_control_3d_pos(float x, float y, float z, uint8_t acc, uint16_t speed);
uint8_t arm_control_to_start_pos(uint8_t acc, uint16_t speed);
void get_current_position(float *x, float *y, float *z);
uint8_t get_arm_target_angles(float x, float y, float z, float *angle1, float *angle2, float *angle3);

#endif /* __STEPPER_ARM_H */
