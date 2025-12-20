#include "chassis_kinematics.h"

void Chassis_Omni_Inverse_Kinematics(float vx, float vy, float vw, float out_rpm[4]) {
    // 基础几何参数
    const float K = CHASSIS_WHEEL_OFFSET_K;

    // 这样能保证逻辑一致性：ry推上去是正vx，但底层执行负vx
    float _vx = -vx;
    float _vy = -vy;
    float _vw = -vw;

    // 1. 重新计算矢量 (45度安装模型)
    float v_lf =  _vx + _vy + _vw * K; // 左前
    float v_rf = -_vx + _vy + _vw * K; // 右前
    float v_rb = -_vx - _vy + _vw * K; // 右后
    float v_lb =  _vx - _vy + _vw * K; // 左后

    // 2. 映射到你的 ID 顺序: 1-左前, 2-右后, 3-右前, 4-左后
    out_rpm[0] = v_lf * MOTOR_RPM_TO_VECTOR;
    out_rpm[1] = v_rb * MOTOR_RPM_TO_VECTOR;
    out_rpm[2] = v_rf * MOTOR_RPM_TO_VECTOR;
    out_rpm[3] = v_lb * MOTOR_RPM_TO_VECTOR;
}