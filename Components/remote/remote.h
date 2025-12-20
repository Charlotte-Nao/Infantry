//
// Created by 14717 on 2025/8/28.
//

#ifndef INFANTRY_01_REMOTE_H
#define INFANTRY_01_REMOTE_H

#include "../../Application/struct_typedef.h"
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