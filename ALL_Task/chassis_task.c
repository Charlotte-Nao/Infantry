#include "../ALL_Task/chassis_task.h"
#include "../Components/motor/motor.h"
#include "../Bsp/uart/bsp_uart.h"
#include "../Application/robot_global.h"
#include "math.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "../Bsp/LED/bsp_LED.h"

/* --- 逻辑常量与控制参数 --- */
#define GIMBAL_YAW_SENS         0.010f
#define GIMBAL_PIT_SENS         0.002f
#define MOUSE_YAW_SENS          0.0004f  // 鼠标横向灵敏度
#define MOUSE_PIT_SENS          0.0002f  // 鼠标纵向灵敏度
#define FOLLOW_P_GAIN           0.5f
#define RC_DEADZONE             10
#define YAW_CENTER_OFFSET       0.0f

// 底盘几何参数配置
#define MOTOR_RPM_TO_VECTOR     (180.0f * 268.0f / 17.0f)
#define CHASSIS_MAX_RAD         6.28 / 50.0f

/* --- 静态控制变量 --- */
static float world_yaw_target = 0.0f;
static float world_pit_target = 0.0f;
static float vx_ramp = 0.0f, vy_ramp = 0.0f;

static uint32_t last_rc_tick = 0;

static float Rad_Format(float angle) {
    while (angle >  M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void chassis_task_func(void const * argument) {
    /******************************************************************************************************************/
    /* 初始化 */
    // 调试串口
    struct uart_device* Uart = uart_get_device("uart1_dma");
    Uart->Init(Uart, 115200, 8, 'N', 1);


    // 获取电机设备句柄
    struct motor_device *chassis[4];
    for(int i=0; i<4; i++) {
        char name[25]; sprintf(name, "M3508_CHASSIS_%d", i+1);
        chassis[i] = motor_get_device(name);
    }
    struct motor_device *yaw_m = motor_get_device("GM6020_YAW");

    // 获取控制设别句柄
    const RC_ctrl_t *rc = robot_ctrl.rc;

    // 按键上升沿检测变量
    static uint8_t last_toggle_cmd = 0;

    // 轮组电机目标值初始化
    float wheel_targets[4] = {0};

    /******************************************************************************************************************/
    // 系统启动保护
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);

    /******************************************************************************************************************/
    // 主循环
    while (1) {
        uint32_t current_tick = osKernelSysTick();

        /**************************************************************************************************************/
        if (current_tick - rc->last_update_tick > 200) {
            // 遥控器掉线保护
            robot_ctrl.monitor.remote_online = 0;
            robot_ctrl.chassis_mode = CHASSIS_RELAX;
        } else {
            robot_ctrl.monitor.remote_online = 1;
            /**********************************************************************************************************/
            // 底盘模式切换
            /* 输入抽象*/
            uint8_t toggle_cmd = (rc->key.v & KEY_CTRL) || rc->rc.custom_r;

            /* 边缘检测 (上升沿触发切换) */
            uint8_t toggle_trigger = (toggle_cmd && !last_toggle_cmd);

            if (toggle_trigger) {
                if (robot_ctrl.chassis_mode != CHASSIS_RELAX) {
                    robot_ctrl.chassis_mode = CHASSIS_RELAX;
                } else {
                    robot_ctrl.chassis_mode = CHASSIS_FOLLOW;
                }
            }

            // 更新状态记录
            last_toggle_cmd = toggle_cmd;
        }

        /**************************************************************************************************************/

        if (robot_ctrl.monitor.remote_online) {
            if (robot_ctrl.chassis_mode != CHASSIS_RELAX) {
                if (robot_ctrl.chassis_mode == CHASSIS_FOLLOW) {
                    LED_RED_RESET();
                    LED_BLUE_RESET();
                    LED_GREEN_SET();

                    // --- A. 解析遥控器数据 ---
                    float vx_rc = (abs(robot_ctrl.rc->rc.ch[0]) > RC_DEADZONE) ? robot_ctrl.rc->rc.ch[0] / 660.0f : 0;
                    float vy_rc = (abs(robot_ctrl.rc->rc.ch[1]) > RC_DEADZONE) ? robot_ctrl.rc->rc.ch[1] / 660.0f : 0;
                    float vw_rc = (abs(robot_ctrl.rc->rc.wheel) > RC_DEADZONE) ? robot_ctrl.rc->rc.wheel / 660.0f : 0;

                    // --- B. 各向同性平移处理 (防除零) ---
                    float v_sum = sqrtf(vx_rc * vx_rc + vy_rc * vy_rc);
                    if (v_sum > 1.0f) {
                        // 只有总矢量超过 1.0 时才缩放，保证小范围控制感
                        vx_rc /= v_sum;
                        vy_rc /= v_sum;
                    }

                    // --- C. 跟随与自由旋转逻辑 ---
                    float vw_final = 0;
                    float yaw_m_pos; // 这里的 POS 应该是云台相对于底盘的相对编码器角度
                    yaw_m->get_status(yaw_m, "POS", &yaw_m_pos);

                    // 角度归一化函数，确保在 -PI 到 PI 之间 (假设 YAW_CENTER_OFFSET 是对中位)
                    float angle_error = Rad_Format(yaw_m_pos - YAW_CENTER_OFFSET);

                    if (fabsf(vw_rc) > 0.05f) {
                        // 1. 如果拨轮在动，执行自由旋转
                        vw_final = vw_rc;
                    } else {
                        // 2. 如果拨轮松开，底盘自动跟随云台 (使用比例控制对齐)
                        // FOLLOW_P_GAIN 通常取 1.5 ~ 4.0 之间
                        vw_final = -angle_error * FOLLOW_P_GAIN;
                    }
                    robot_ctrl.chassis.yaw_speed = vw_final * CHASSIS_MAX_RAD;

                    // --- D. 随动坐标系变换 (让底盘 W A S D 始终相对于云台方向) ---
                    // 如果不加这个，底盘旋转时 W 就不是朝向云台指的方向了
                    float final_vx = vx_rc * cosf(angle_error) - vy_rc * sinf(angle_error);
                    float final_vy = vx_rc * sinf(angle_error) + vy_rc * cosf(angle_error);

                    // --- E. 逆运动学计算 ---
                    wheel_targets[0] = (-final_vx - final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                    wheel_targets[1] = (-final_vx + final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                    wheel_targets[2] = (final_vx + final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                    wheel_targets[3] = (final_vx - final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;

                    // 写入目标
                    for (int i = 0; i < 4; i++) {
                        if (chassis[i]) chassis[i]->set_target(chassis[i], 1, wheel_targets[i]);
                    }
                }
            }
            else {
                LED_RED_SET();
                LED_BLUE_RESET();
                LED_GREEN_RESET();
            }
        }
        else {
            // 遥控器掉线：红灯快闪
            LED_GREEN_RESET();
            LED_BLUE_RESET();
            LED_RED_Toggle();
            osDelay(100);
        }


        osDelay(2);
    }
}