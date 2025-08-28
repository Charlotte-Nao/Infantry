//
// Created by 14717 on 2025/8/28.
//

#ifndef INFANTRY_01_REMOTE_H
#define INFANTRY_01_REMOTE_H

#include "../../Application/struct_typedef.h"

struct remote_device {
    char *name;
    void (*init)(struct remote_device *pDev);
    void (*unable)(struct remote_device *pDev);
    void (*restart)(struct remote_device *pDev, uint16_t dma_buf_num);
    void *remote_data;
};

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

extern struct remote_device *remote_get_device(const char *name);

extern const RC_ctrl_t *get_remote_control_point(void);
#endif //INFANTRY_01_REMOTE_H