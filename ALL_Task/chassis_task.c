#include "../ALL_Task/chassis_task.h"   // 底盘任务头文件
#include "../Components/motor/motor.h"  // 电机驱动头文件
#include "../Bsp/uart/bsp_uart.h"       // 串口驱动头文件
#include "../Application/robot_global.h"// 全局变量头文件
#include "math.h"                       // 数学库-三角函数/开方/绝对值运算
#include "stdlib.h"                     // 标准库头文件
#include "cmsis_os.h"                   // RTOS系统库-系统滴答/延时
#include "stdio.h"                      // 标准输入输出-sprintf使用

/* --- 底盘控制 核心有效宏定义【仅保留实际被调用的参数，全部有用，无冗余】 --- */
#define FOLLOW_P_GAIN           0.5f    // 底盘跟随云台 比例系数(PID-P)，核心跟随参数
#define RC_DEADZONE             10      // 遥控器摇杆死区阈值，过滤摇杆漂移无效信号
#define YAW_CENTER_OFFSET       0.0f    // 云台航向角 中心零点偏移补偿值
#define MOTOR_RPM_TO_VECTOR     (180.0f * 268.0f / 17.0f)  // 转速→底盘运动矢量 换算系数
#define CHASSIS_MAX_RAD         6.28 / 50.0f               // 底盘最大旋转角速度限制，硬件保护

/***********************************************************************************************************************
* 函数名：Rad_Format
* 功  能：弧度角度归一化，约束角度在 [-π, π] 区间内，防止角度跳变
* 参  数：angle - 待归一化的原始弧度值
* 返回值：归一化后的合规弧度值
***********************************************************************************************************************/
static float Rad_Format(float angle) {
    while (angle >  M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

/***********************************************************************************************************************
* 函数名：chassis_task_func
* 功  能：底盘控制任务主函数(2ms调度周期)，核心实现底盘失能/云台跟随双模式控制
* 核心逻辑：遥控器掉线急停保护 → 模式切换防抖 → 摇杆+键盘输入融合 → 速度限速 → 云台跟随计算 → 坐标系变换 → 逆运动学解算 → 电机下发
***********************************************************************************************************************/
void chassis_task_func(void const * argument) {
    /******************************************************************************************************************/
    /* 硬件外设初始化区 - 仅执行一次 */
    struct uart_device* Uart = uart_get_device("uart1_dma");
    Uart->Init(Uart, 115200, 8, 'N', 1);

    // 获取4个底盘M3508电机句柄
    struct motor_device *chassis[4];
    for(int i=0; i<4; i++) {
        char name[25];
        sprintf(name, "M3508_CHASSIS_%d", i+1);
        chassis[i] = motor_get_device(name);
    }
    // 获取云台航向轴GM6020电机句柄（读取云台角度用）
    struct motor_device *yaw_m = motor_get_device("GM6020_YAW");
    const RC_ctrl_t *rc = robot_ctrl.rc;

    // 按键防抖变量+电机目标值数组
    static uint8_t last_toggle_cmd = 0;
    float wheel_targets[4] = {0};

    /******************************************************************************************************************/
    // 系统上电启动保护：等待传感器就绪+外设稳定
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);

    /******************************************************************************************************************/
    // 底盘任务主循环
    while (1) {
        uint32_t current_tick = osKernelSysTick();

        /**************************************************************************************************************/
        // 最高优先级：遥控器掉线保护 → 强制进入失能模式
        if (current_tick - rc->last_update_tick > 200) {
            robot_ctrl.monitor.remote_online = 0;
            robot_ctrl.chassis_mode = CHASSIS_RELAX;
        }
        else {
            robot_ctrl.monitor.remote_online = 1;

            // 底盘模式切换：失能模式 ↔ 云台跟随模式 (上升沿防抖触发)
            uint8_t toggle_cmd = (rc->key.v & KEY_CTRL) || rc->rc.custom_r;
            uint8_t toggle_trigger = (toggle_cmd && !last_toggle_cmd);
            if (toggle_trigger) {
                robot_ctrl.chassis_mode = (robot_ctrl.chassis_mode != CHASSIS_RELAX) ? CHASSIS_RELAX : CHASSIS_FOLLOW;
            }
            last_toggle_cmd = toggle_cmd;
        }

        /**************************************************************************************************************/
        // 遥控器在线时的底盘控制逻辑
        if (robot_ctrl.monitor.remote_online) {
            // 云台跟随模式：完整控制逻辑执行
            if (robot_ctrl.chassis_mode == CHASSIS_FOLLOW) {
                // 遥控器摇杆输入解析+死区过滤+归一化
                float vx_rc = (abs(rc->rc.ch[0]) > RC_DEADZONE) ? rc->rc.ch[0] / 660.0f : 0;
                float vy_rc = (abs(rc->rc.ch[1]) > RC_DEADZONE) ? rc->rc.ch[1] / 660.0f : 0;
                float vw_rc = (abs(rc->rc.wheel) > RC_DEADZONE) ? rc->rc.wheel / 660.0f : 0;

                // 键盘输入解析 + Shift变速逻辑
                float vx_kb = 0, vy_kb = 0, vw_kb = 0;
                float speed_ratio = (rc->key.v & KEY_SHIFT) ? 1.0f : 0.5f;
                if (rc->key.v & KEY_W) vy_kb += speed_ratio;
                if (rc->key.v & KEY_S) vy_kb -= speed_ratio;
                if (rc->key.v & KEY_A) vx_kb -= speed_ratio;
                if (rc->key.v & KEY_D) vx_kb += speed_ratio;
                if (rc->key.v & KEY_Q) vw_kb -= 0.5f;
                if (rc->key.v & KEY_E) vw_kb += 0.5f;

                // 摇杆+键盘输入融合
                float total_vx = vx_rc + vx_kb;
                float total_vy = vy_rc + vy_kb;

                // 各向同性限速：保证全方位移动速度一致
                float v_norm = sqrtf(total_vx * total_vx + total_vy * total_vy);
                if (v_norm > speed_ratio) {
                    total_vx /= v_norm * speed_ratio;
                    total_vy /= v_norm * speed_ratio;
                }

                // 读取云台角度+归一化角度偏差
                float yaw_m_pos;
                yaw_m->get_status(yaw_m, "POS", &yaw_m_pos);
                float angle_error = Rad_Format(yaw_m_pos - YAW_CENTER_OFFSET);

                // 手动旋转优先于云台自动跟随
                float vw_final = 0;
                if (fabsf(vw_rc) > 0.05f || fabsf(vw_kb) > 0.01f) {
                    vw_final = vw_rc + vw_kb;
                } else {
                    vw_final = -angle_error * FOLLOW_P_GAIN;
                }
                robot_ctrl.chassis.yaw_speed = vw_final * CHASSIS_MAX_RAD;

                // 云台坐标系变换：WASD以云台朝向为基准
                float final_vx = total_vx * cosf(angle_error) - total_vy * sinf(angle_error);
                float final_vy = total_vx * sinf(angle_error) + total_vy * cosf(angle_error);

                // 麦克纳姆轮逆运动学解算
                wheel_targets[0] = (-final_vx - final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                wheel_targets[1] = (-final_vx + final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                wheel_targets[2] = (final_vx + final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                wheel_targets[3] = (final_vx - final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;

                // 下发目标转速到4个底盘电机
                for (int i = 0; i < 4; i++) {
                    if (chassis[i]) chassis[i]->set_target(chassis[i], 1, wheel_targets[i]);
                }
            }
            // 失能模式：所有底盘电机目标转速置0，无动力输出
            else if (robot_ctrl.chassis_mode == CHASSIS_RELAX) {
                for (int i = 0; i < 4; i++) {
                    if (chassis[i]) chassis[i]->set_target(chassis[i], 1, 0);
                }
            }
        }
        // 遥控器掉线：所有电机强制置0，防止失控漂移
        else {
            for (int i = 0; i < 4; i++) {
                if (chassis[i]) chassis[i]->set_target(chassis[i], 1, 0);
            }
            osDelay(100);
        }
        osDelay(2);
    }
}