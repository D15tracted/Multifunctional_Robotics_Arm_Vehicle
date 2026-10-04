#include "stm32f4xx_hal.h"
#include "PS4_Arm.h"

/* Flag bits for each button */
static uint8_t ps4_btn_flag[PS4_BTN_NUM];
static uint8_t last_state[PS4_BTN_NUM];

/**
 * @brief Initialize PS4 button states
 */
void PS4_BtnInit(void)
{
    for (uint8_t i = 0; i < PS4_BTN_NUM; i++) {
        ps4_btn_flag[i] = 0;
        last_state[i] = 0;
    }
}

/**
 * @brief Update button states and flags according to the Bluetooth data array
 * @param data Array of length 16, arranged as in the specification
 *             Recommended call frequency: every 10–20 ms
 */
void PS4_BtnTick(uint8_t *data)
{
    uint8_t curr;

    /* ---------- ???? UP ? DOWN(????????) ---------- */
    /* UP (dpadUp) : index 13 */
    curr = (data[13] != 0) ? 1 : 0;
    if (curr != last_state[PS4_BTN_UP]) {
        if (curr == 1) {
            ps4_btn_flag[PS4_BTN_UP] |= PS4_BTN_DOWN;
        } else {
            ps4_btn_flag[PS4_BTN_UP] |= PS4_BTN_UP;
        }
        last_state[PS4_BTN_UP] = curr;
    }
    if (curr) {
        ps4_btn_flag[PS4_BTN_UP] |= PS4_BTN_HOLD;
    } else {
        ps4_btn_flag[PS4_BTN_UP] &= ~PS4_BTN_HOLD;
    }

    /* DOWN (dpadDown) : index 15 */
    curr = (data[15] != 0) ? 1 : 0;
    if (curr != last_state[PS4_BTN_DOWN]) {
        if (curr == 1) {
            ps4_btn_flag[PS4_BTN_DOWN] |= PS4_BTN_DOWN;
        } else {
            ps4_btn_flag[PS4_BTN_DOWN] |= PS4_BTN_UP;
        }
        last_state[PS4_BTN_DOWN] = curr;
    }
    if (curr) {
        ps4_btn_flag[PS4_BTN_DOWN] |= PS4_BTN_HOLD;
    } else {
        ps4_btn_flag[PS4_BTN_DOWN] &= ~PS4_BTN_HOLD;
    }

    /* ---------- ????????(???????) ---------- */
    for (uint8_t i = 0; i < PS4_BTN_NUM; i++) {
        /* ?????????? UP ? DOWN */
        if (i == PS4_BTN_UP || i == PS4_BTN_DOWN) {
            continue;
        }

        /* ???????? */
        if (i == PS4_BTN_R2) {
            if (data[4] > 75) {
                curr = 1;
            } else if (data[4] < 25) {
                curr = 0;
            } else {
                curr = last_state[PS4_BTN_R2];
            }
        } else if (i == PS4_BTN_L2) {
            if (data[5] > 75) {
                curr = 1;
            } else if (data[5] < 25) {
                curr = 0;
            } else {
                curr = last_state[PS4_BTN_L2];
            }
        } else {
            uint8_t idx;
            if      (i == PS4_BTN_A)     idx = 6;
            else if (i == PS4_BTN_B)     idx = 7;
            else if (i == PS4_BTN_X)     idx = 8;
            else if (i == PS4_BTN_Y)     idx = 9;
            else if (i == PS4_BTN_R1)    idx = 10;
            else if (i == PS4_BTN_L1)    idx = 11;
            else if (i == PS4_BTN_LEFT)  idx = 12;
            else if (i == PS4_BTN_RIGHT) idx = 14;
            else continue;

            curr = (data[idx] != 0) ? 1 : 0;
        }

        /* ???? */
        if (curr != last_state[i]) {
            if (curr == 1) {
                ps4_btn_flag[i] |= PS4_BTN_DOWN;
            } else {
                ps4_btn_flag[i] |= PS4_BTN_UP;
            }
            last_state[i] = curr;
        }

        /* ?? HOLD ?? */
        if (curr) {
            ps4_btn_flag[i] |= PS4_BTN_HOLD;
        } else {
            ps4_btn_flag[i] &= ~PS4_BTN_HOLD;
        }
    }
}

/**
 * @brief Check a button flag (transient flags are cleared after read, except HOLD)
 * @param btn  Button index
 * @param flag Flag to check (PS4_BTN_DOWN / PS4_BTN_UP / PS4_BTN_HOLD)
 * @return 1 if the flag is set, otherwise 0
 */
uint8_t PS4_BtnCheck(uint8_t btn, uint8_t flag)
{
    if (btn >= PS4_BTN_NUM) return 0;

    if (ps4_btn_flag[btn] & flag) {
        if (flag != PS4_BTN_HOLD) {
            ps4_btn_flag[btn] &= ~flag;   /* Clear transient flag after read */
        }
        return 1;
    }
    return 0;
}
