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
        // 只修改：遥控器掉线检测 → VT13专属超时判定
        if (current_tick - rc->vt13.last_update_tick > 200) {
            // 遥控器掉线保护
            robot_ctrl.monitor.remote_online = 0;
            robot_ctrl.chassis_mode = CHASSIS_RELAX;
        } else {
            robot_ctrl.monitor.remote_online = 1;
            /**********************************************************************************************************/
            // 底盘模式切换
            /* 输入抽象*/
            // 只修改：CTRL键盘按键+自定义右键 → VT13专属按键
            uint8_t toggle_cmd = (rc->vt13.key_vt13.v & KEY_VT13_CTRL) || rc->vt13.rc_vt13.custom_r;

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

                    // --- A. 输入源融合 (遥控器摇杆 + 键盘) ---
                    // 只修改：右摇杆CH0/CH1 + 拨轮 → VT13专属通道
                    float vx_rc = (abs(rc->vt13.rc_vt13.ch[0]) > RC_DEADZONE) ? rc->vt13.rc_vt13.ch[0] / 660.0f : 0;
                    float vy_rc = (abs(rc->vt13.rc_vt13.ch[1]) > RC_DEADZONE) ? rc->vt13.rc_vt13.ch[1] / 660.0f : 0;
                    float vw_rc = (abs(rc->vt13.rc_vt13.wheel) > RC_DEADZONE) ? rc->vt13.rc_vt13.wheel / 660.0f : 0;

                    float vx_kb = 0, vy_kb = 0, vw_kb = 0;
                    // 只修改：Shift变速+WASD+QE键盘 → VT13专属键盘按键
                    float speed_ratio = (rc->vt13.key_vt13.v & KEY_VT13_SHIFT) ? 1.0f : 0.5f;

                    if (rc->vt13.key_vt13.v & KEY_VT13_W) vy_kb += speed_ratio;
                    if (rc->vt13.key_vt13.v & KEY_VT13_S) vy_kb -= speed_ratio;
                    if (rc->vt13.key_vt13.v & KEY_VT13_A) vx_kb -= speed_ratio;
                    if (rc->vt13.key_vt13.v & KEY_VT13_D) vx_kb += speed_ratio;
                    if (rc->vt13.key_vt13.v & KEY_VT13_Q) vw_kb -= 0.5f; // 手动左旋
                    if (rc->vt13.key_vt13.v & KEY_VT13_E) vw_kb += 0.5f; // 手动右旋

                    float total_vx = vx_rc + vx_kb;
                    float total_vy = vy_rc + vy_kb;

                    // --- B. 各向同性限速 ---
                    float v_norm = sqrtf(total_vx * total_vx + total_vy * total_vy);
                    if (v_norm > speed_ratio) {
                        total_vx = total_vx / v_norm * speed_ratio;
                        total_vy = total_vy / v_norm * speed_ratio;
                    }

                    // --- C. 跟随与旋转逻辑 ---
                    float yaw_m_pos;
                    yaw_m->get_status(yaw_m, "POS", &yaw_m_pos);
                    float angle_error = Rad_Format(yaw_m_pos - YAW_CENTER_OFFSET);

                    float vw_final = 0;
                    // 键盘 QE 或 摇杆拨轮 优先于 自动跟随
                    if (fabsf(vw_rc) > 0.05f || fabsf(vw_kb) > 0.01f) {
                        vw_final = vw_rc + vw_kb;
                    } else {
                        vw_final = -angle_error * FOLLOW_P_GAIN;
                    }
                    robot_ctrl.chassis.yaw_speed = vw_final * CHASSIS_MAX_RAD;

                    // --- D. 随动坐标系变换 (WASD 以云台为基准) ---
                    float final_vx = total_vx * cosf(angle_error) - total_vy * sinf(angle_error);
                    float final_vy = total_vx * sinf(angle_error) + total_vy * cosf(angle_error);

                    // --- E. 逆运动学计算 ---
                    wheel_targets[0] = (final_vx + final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                    wheel_targets[1] = (final_vx - final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                    wheel_targets[2] = (-final_vx - final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;
                    wheel_targets[3] = (-final_vx + final_vy - vw_final) * MOTOR_RPM_TO_VECTOR;

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