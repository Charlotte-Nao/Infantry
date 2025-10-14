#include "auto_aim.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

// 全局变量
static struct uart_device *auto_aim_uart = NULL;

// 初始化自瞄系统
int auto_aim_init(struct uart_device *uart_dev) {
    if (uart_dev == NULL) {
        return -1;
    }

    auto_aim_uart = uart_dev;

    return 0;
}

// 解析目标数据

int parse_target_data(target_info_t *target) {
    char buffer[100];
    int received_len = auto_aim_uart->Recv(auto_aim_uart, buffer, sizeof(buffer) - 1, 200);

    if (received_len > 0) {
        buffer[received_len-1] = '\0';

        // 检查数据头
        if (strncmp(buffer, "ARMOR,", 6) != 0) {
            return -1;
        }

        // 分步解析
        char *token;
        char *rest = buffer;

        // 解析yaw
        // 解析 yaw
        token = strtok_r(rest, ",", &rest);
        if (!token) return -1;
        target->aim_target_yaw = strtof(token, NULL);

        // 解析 pitch
        token = strtok_r(NULL, ",\n", &rest); // 也以换行符作为分隔符
        if (!token) return -1;
        target->aim_target_pitch = strtof(token, NULL);

        //auto_aim_uart->Print(auto_aim_uart, "%d\r\n", is_target_valid(target));
        return is_target_valid(target);
    }

    return 0;
}

// 判断目标是否有效
int is_target_valid(target_info_t *target) {
    // 检查目标坐标是否合理（简单检查）
    if (isnan(target->aim_target_pitch) || isnan(target->aim_target_yaw)) {
        return 0;
    }
    return 1;
}

void auto_aim_control(target_info_t *target, float *yaw_output, float *pitch_output) {
    *yaw_output = target->aim_target_yaw;
    *pitch_output = target->aim_target_pitch;
}

