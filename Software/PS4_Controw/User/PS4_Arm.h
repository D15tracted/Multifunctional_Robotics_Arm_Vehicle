#ifndef __PS4_ARM_H
#define __PS4_ARM_H

#include <stdint.h>

/* Button index enumeration */
enum {
    PS4_BTN_R2 = 0,
    PS4_BTN_L2,
    PS4_BTN_A,
    PS4_BTN_B,
    PS4_BTN_X,
    PS4_BTN_Y,
    PS4_BTN_R1,
    PS4_BTN_L1,
    PS4_BTN_LEFT,
    PS4_BTN_UP,
    PS4_BTN_RIGHT,
    PS4_BTN_DOWN,
    PS4_BTN_NUM
};

/* Button flag bits (similar to Key_Flag) */
#define PS4_BTN_DOWN    0x01   /* Button pressed edge */
#define PS4_BTN_UP      0x02   /* Button released edge */
#define PS4_BTN_HOLD    0x04   /* Button being held */

void PS4_BtnInit(void);
void PS4_BtnTick(uint8_t *data);
uint8_t PS4_BtnCheck(uint8_t btn, uint8_t flag);

#endif
