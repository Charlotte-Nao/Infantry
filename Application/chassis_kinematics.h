#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

#include "main.h"

// 底盘几何参数配置 (根据机器人实际尺寸修改)
#define CHASSIS_WHEEL_OFFSET_K  1.0f     // 轮子中心到机器人中心的几何系数
#define MOTOR_RPM_TO_VECTOR     3500.0f  // 速度矢量到电机转速的映射系数

/**
 * @brief 底盘运动状态结构体
 */
typedef struct {
    float vx; // 纵向速度 m/s
    float vy; // 横向速度 m/s
    float vw; // 自旋角速度 rad/s
} Chassis_Speed_t;

/**
 * @brief 全向轮底盘逆解：将速度矢量分解为四个电机的目标转速
 */
void Chassis_Omni_Inverse_Kinematics(float vx, float vy, float vw, float out_rpm[4]);

#endif