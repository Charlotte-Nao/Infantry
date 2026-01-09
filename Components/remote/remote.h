//
// Created by 14717 on 2025/8/28.
//

#ifndef INFANTRY_01_REMOTE_H
#define INFANTRY_01_REMOTE_H

#include "../../Application/struct_typedef.h"

/**********************************************************************************************************************/

#define SBUS_RX_BUF_NUM 36u   /*缓存区长度*/
#define RC_FRAME_LENGTH 18u   /*有效数据长度*/

/**********************************************************************************************************************/
/*遥控器摇杆定义*/

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/**********************************************************************************************************************/
/*遥控器拨杆定义*/

#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

/**********************************************************************************************************************/
/*键值定义*/

#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

/**********************************************************************************************************************/
/*控制信息结构体定义*/

typedef struct __attribute__((packed))
{
    struct __attribute__((packed))
    {
        int16_t ch[5];
        char s[2];
    } rc;

    struct __attribute__((packed))
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    struct __attribute__((packed))
    {
        uint16_t v;
    } key;

} RC_ctrl_t;

/**********************************************************************************************************************/
/*函数声明*/
extern void RC_init(void);

extern void RC_unable(void);

extern void RC_restart(uint16_t dma_buf_num);

extern const RC_ctrl_t *RC_get_handle(void);

#endif //INFANTRY_01_REMOTE_H