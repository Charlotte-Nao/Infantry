#ifndef AUTOAIM_H
#define AUTOAIM_H


#include "../Driver/uart/dvc_uart.h"
#include "../Driver/usb_cdc/usb_cdc.h"
#include <stdint.h>



// 目标信息结构体
typedef struct {
    float aim_target_yaw;
    float aim_target_pitch;
} target_info_t;

// 函数声明
int auto_aim_init(struct usb_device *uart_dev);
int parse_target_data(target_info_t *target);
int is_target_valid(target_info_t *target);
void auto_aim_control(target_info_t *target, float *yaw_output, float *pitch_output);

#endif //AUTOAIM_H