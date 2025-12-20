#ifndef INFANTRY_01_MOTOR_H
#define INFANTRY_01_MOTOR_H

#include "../../Application/struct_typedef.h"
#include "stm32f4xx_hal.h"

struct motor_device {
    // 成员变量
    char *motor_name;
    uint32_t motor_id;              // 反馈帧对应的 StdId (大疆: 0x201+; 达妙: 0x00)
    CAN_HandleTypeDef* motor_can_handle;
    void *motor_data;               // 指向具体电机数据结构体 (如 DM_MIT_data)

    // --- 函数指针接口 ---

    // 初始化
    void (*init)(struct motor_device *motor, uint32_t motor_ID, CAN_HandleTypeDef *hcan, int para_num, ...);

    // 反馈解析
    void (*get_measure)(const struct motor_device *motor, const uint8_t *data);

    // 状态更新 (PID计算等)
    void (*update)(struct motor_device *motor);

    // 动作指令
    void (*send_enable_cmd)(struct motor_device *motor);
    void (*send_disable_cmd)(struct motor_device *motor);
    void (*send_ctrl_cmd)(struct motor_device *motor);

    // 数据交互
    void (*set_target)(const struct motor_device *motor, const int para_num, ...);
    void (*get_status)(const struct motor_device *motor, const char* which_status, void* status_data);
    void (*set_para)(const struct motor_device *motor, const char* which_para, void* para_data);
};

/* --- 电机控制核心接口 --- */

// 实例查找
struct motor_device *motor_get_device(const char *name);

// 系统级调用
void Motor_System_PowerOn_Init(void); // 上电初始化所有电机
void Motor_All_Update(void);          // 在控制循环中计算所有电机PID

/**
 * @brief 大疆电机组帧发送函数 (按总线拆分)
 */
void DJI_Motor_Send_CAN1_Group(CAN_HandleTypeDef *hcan);
void DJI_Motor_Send_CAN2_Group(CAN_HandleTypeDef *hcan);

// 硬件句柄声明
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#endif //INFANTRY_01_MOTOR_H