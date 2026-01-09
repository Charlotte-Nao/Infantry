#ifndef REMOTE_H
#define REMOTE_H

#include "../../Application/struct_typedef.h"

/* --- 基础协议常量 --- */
#define RC_FRAME_LENGTH      21u
#define RC_RX_BUF_SIZE       (RC_FRAME_LENGTH * 2)
#define RC_CH_VALUE_OFFSET   ((uint16_t)1024)  // 中间值偏移

/* --- 摇杆/拨轮 通道限幅 --- */
#define RC_CH_MIN            364
#define RC_CH_MID            1024
#define RC_CH_MAX            1684
#define RC_CH_RANGE          660   // (1684 - 1024)

/* --- 挡位切换开关 (mode_sw) --- */
#define RC_SW_C              ((uint8_t)0)  // 下 (对应挡位C)
#define RC_SW_N              ((uint8_t)1)  // 中 (对应挡位N)
#define RC_SW_S              ((uint8_t)2)  // 上 (对应挡位S)

/* --- 状态逻辑定义 (按键类) --- */
#define RC_BTN_UP            ((uint8_t)0)  // 未按下
#define RC_BTN_DOWN          ((uint8_t)1)  // 按下

/* --- 键盘按键位映射 (Bitmask) --- */
#define KEY_W                ((uint16_t)0x0001) // bit 0
#define KEY_S                ((uint16_t)0x0002) // bit 1
#define KEY_A                ((uint16_t)0x0004) // bit 2
#define KEY_D                ((uint16_t)0x0008) // bit 3
#define KEY_SHIFT            ((uint16_t)0x0010) // bit 4
#define KEY_CTRL             ((uint16_t)0x0020) // bit 5
#define KEY_Q                ((uint16_t)0x0040) // bit 6
#define KEY_E                ((uint16_t)0x0080) // bit 7
#define KEY_R                ((uint16_t)0x0100) // bit 8
#define KEY_F                ((uint16_t)0x0200) // bit 9
#define KEY_G                ((uint16_t)0x0400) // bit 10
#define KEY_Z                ((uint16_t)0x0800) // bit 11
#define KEY_X                ((uint16_t)0x1000) // bit 12
#define KEY_C                ((uint16_t)0x2000) // bit 13
#define KEY_V                ((uint16_t)0x4000) // bit 14
#define KEY_B                ((uint16_t)0x8000) // bit 15

/* --- 原始数据结构体 (21 Bytes) --- */
typedef struct __attribute__((packed)) {
    uint8_t sof_1;          // 0xA9
    uint8_t sof_2;          // 0x53
    uint64_t ch_0:11;       // 右水平
    uint64_t ch_1:11;       // 右竖直
    uint64_t ch_2:11;       // 左竖直
    uint64_t ch_3:11;       // 左水平
    uint64_t mode_sw:2;     // 挡位 C/N/S
    uint64_t btn_pause:1;   // 暂停
    uint64_t btn_custom_l:1;// 自定义左
    uint64_t btn_custom_r:1;// 自定义右
    uint64_t wheel:11;      // 拨轮
    uint64_t btn_trigger:1; // 扳机

    int16_t mouse_x;        // 鼠标左右增量
    int16_t mouse_y;        // 鼠标前后增量
    int16_t mouse_z;        // 鼠标滚轮增量
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key_v;         // 键盘按键位图
    uint16_t crc16;
} remote_raw_t;

/* --- 逻辑控制结构体 --- */
typedef struct {
    struct {
        int16_t ch[4];      // 摇杆 [-660, 660]
        uint8_t sw;         // 挡位 0, 1, 2
        uint8_t pause;      // 暂停按键 0, 1
        uint8_t custom_l;   // 自定义左 0, 1
        uint8_t custom_r;   // 自定义右 0, 1
        int16_t wheel;      // 拨轮 [-660, 660]
        uint8_t trigger;    // 扳机按键 0, 1
    } rc;
    struct {
        int16_t x, y, z;
        uint8_t press_l, press_r, press_m;
    } mouse;
    struct {
        uint16_t v;         // 键盘位图
    } key;
} RC_ctrl_t;

void RC_Init(void);
const RC_ctrl_t *RC_Get_Handle(void);

#endif