#ifndef ROBOT_GLOBAL_H
#define ROBOT_GLOBAL_H

#include "struct_typedef.h"
#include "stdint.h"
#include "../Components/remote/remote.h"

/* --- 模式枚举定义 --- */

typedef enum {
    GIMBAL_RELAX = 0,    // 失能状态，电机不出力
    GIMBAL_REMOTE,       // 遥控器手动模式（基于IMU控制）
    GIMBAL_AUTO,         // 视觉自瞄模式
    GIMBAL_SEARCH        // 巡检/扫掠模式
} gimbal_mode_e;

typedef enum {
    CHASSIS_RELAX = 0,   // 失能状态
    CHASSIS_STATIC,      // 独立模式（不随动，以底盘坐标系为准）
    CHASSIS_FOLLOW,      // 跟随模式（以云台朝向为正前方）
    CHASSIS_SPIN         // 小陀螺模式（自转）
} chassis_mode_e;

typedef enum {
    SHOOT_STOP = 0,      // 停止发射
    SHOOT_READY,         // 摩擦轮起旋
    SHOOT_FIRE_SINGLE,   // 单发模式
    SHOOT_FIRE_CONTINUE  // 连发模式
} shoot_mode_e;

/* --- 核心控制结构体 --- */

typedef struct {
    // 1. 系统当前运行模式
    gimbal_mode_e  gimbal_mode;
    chassis_mode_e chassis_mode;
    shoot_mode_e   shoot_mode;

    // 2. 姿态反馈数据 (由 Sensor Task 更新)
    struct {
        fp32 yaw;        // 当前航向角 (度)
        fp32 pitch;      // 当前俯仰角 (度)
        fp32 roll;       // 当前横滚角 (度)
        fp32 yaw_v;      // 航向角速度 (度/s)
        fp32 pitch_v;    // 俯仰角速度 (度/s)
    } imu;

    // 3. 相对位置与电机反馈 (由 Control Task 或专门的反馈逻辑更新)
    struct {
        fp32 relative_yaw;      // 云台相对于底盘的机械夹角 (由编码器转化)
        int16_t yaw_motor_msg;  // 云台Yaw电机原始编码器值
        int16_t pit_motor_msg;  // 云台Pitch电机原始编码器值
    } motor_info;

    // 4. 控制目标值 (Setpoint，由 Logic Task 计算，Control Task 使用)
    struct {
        // 底盘目标速度
        fp32 vx;         // 前后向速度
        fp32 vy;         // 左右向速度
        fp32 vw;         // 自转速度

        // 云台目标角度 (由遥控器或自瞄解算)
        fp32 gimbal_yaw;
        fp32 gimbal_pit;
    } target;

    // 5. 裁判系统与物理状态
    struct {
        uint16_t current_power;      // 当前实时功率 (W)
        uint16_t power_buffer;       // 能量缓冲 (J)
        uint16_t shooter_heat;       // 当前枪口热量
        uint16_t shooter_heat_limit; // 枪口热量上限
    } judge_info;

    // 6. 系统监控与异常处理
    struct {
        uint8_t  sensor_ready;   // 传感器校准完成标志
        uint8_t  remote_online;  // 遥控器在线标志
        uint8_t  vision_online;  // 视觉系统在线标志
        uint32_t error_code;     // 错误码记录
    } monitor;

    // 7. 输入引用指针
    const RC_ctrl_t *rc;         // 遥控器原始数据引用
} robot_ctrl_info_t;

/* --- 全局变量声明 --- */
extern robot_ctrl_info_t robot_ctrl;

/* --- 核心工具函数 --- */
void Robot_Global_Init(void);

#endif