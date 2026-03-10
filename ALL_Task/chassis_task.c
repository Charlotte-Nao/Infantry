#include "../ALL_Task/chassis_task.h"
#include "../Components/motor/motor.h"
#include "../Bsp/uart/bsp_uart.h"
#include "../Application/robot_global.h"
#include "math.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "../Bsp/LED/bsp_LED.h"
#include "../Components/referee/referee.h"  // 【新增】引入裁判系统组件

/* --- 逻辑常量与控制参数 --- */
#define GIMBAL_YAW_SENS         0.010f
#define GIMBAL_PIT_SENS         0.002f
#define MOUSE_YAW_SENS          0.0004f  // 鼠标横向灵敏度
#define MOUSE_PIT_SENS          0.0002f  // 鼠标纵向灵敏度
#define FOLLOW_P_GAIN           0.5f
#define RC_DEADZONE             10
#define YAW_CENTER_OFFSET       0.003f

// 底盘几何参数配置
#define MOTOR_RPM_TO_VECTOR     3000.0f
#define CHASSIS_MAX_RAD         MOTOR_RPM_TO_VECTOR / 50.0f
#define rotation_speed          1.0f      //底盘自转转速比例

// 回正相关参数
#define YAW_ALIGN_THRESHOLD     0.05f    // 放宽到位阈值（适配机械误差，约2.86度）
#define WHEEL_ACTIVE_THRESHOLD  0.01f    // 拨轮有效输入阈值
#define QE_ACTIVE_THRESHOLD     0.01f     // Q/E有效输入阈值

/* --- 静态控制变量 --- */
static float world_yaw_target = 0.0f;
static float world_pit_target = 0.0f;
static float vx_ramp = 0.0f, vy_ramp = 0.0f;

static uint32_t last_rc_tick = 0;

// 扩展：新增Q/E相关状态变量，和拨轮统一管理
static uint8_t last_wheel_active = 0;    // 上一帧拨轮是否激活
static uint8_t last_qe_active = 0;       // 上一帧Q/E是否激活
static uint8_t yaw_align_enable = 0;     // 回正使能标志（1=需要回正，0=不需要）
static float last_manual_vw = 0.0f;      // 保存「拨轮/Q/E」松开前的最后有效旋转速度（统一变量，避免冲突）

static float Rad_Format(float angle) {
    while (angle >  M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void chassis_task_func(void const * argument)
{
    /******************************************************************************************************************/
    /* 初始化 */
    struct uart_device* Uart = uart_get_device("uart1_dma");
    Uart->Init(Uart, 115200, 8, 'N', 1);

    struct motor_device *chassis[4];
    for(int i=0; i<4; i++) {
        char name[25]; sprintf(name, "M3508_CHASSIS_%d", i+1);
        chassis[i] = motor_get_device(name);
    }
    struct motor_device *yaw_m = motor_get_device("GM6020_YAW");

    const RC_ctrl_t *rc = robot_ctrl.rc;
    static uint8_t last_toggle_cmd = 0;
    float wheel_targets[4] = {0};

    /******************************************************************************************************************/
    // 系统启动保护
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);

    /******************************************************************************************************************/
    // 主循环
    while (1) {
        uint32_t current_tick = osKernelSysTick();


        // ==========================================================
        // 【新增 1】裁判系统仪表盘打印 (每 500ms 打印一次)
        // 函数内部自带限频锁，直接调用即可，绝对不会阻塞 RTOS
        // ==========================================================
        Referee_Debug_Print();

        // ==========================================================
        // 【新增 2】裁判系统 CAN 数据转发 (限制为 50Hz，即每 20ms 发送一次)
        // ==========================================================
        static uint32_t last_can_send_tick = 0;
        if (current_tick - last_can_send_tick >= 20) {
            Referee_CAN_Forward();
            last_can_send_tick = current_tick;
        }


        /**************************************************************************************************************/
        // 遥控器掉线检测
        if (current_tick - rc->vt13.last_update_tick > 200) {
            robot_ctrl.monitor.remote_online = 0;
            robot_ctrl.chassis_mode = CHASSIS_RELAX;

            // 掉线时重置所有标志和保存的速度
            yaw_align_enable = 0;
            last_wheel_active = 0;
            last_qe_active = 0;
            last_manual_vw = 0.0f;
        } else {
            robot_ctrl.monitor.remote_online = 1;
            /**********************************************************************************************************/
            // 底盘模式切换
            uint8_t toggle_cmd = (rc->vt13.key_vt13.v & KEY_VT13_CTRL) || rc->vt13.rc_vt13.custom_r;
            uint8_t toggle_trigger = (toggle_cmd && !last_toggle_cmd);

            if (toggle_trigger) {
                if (robot_ctrl.chassis_mode != CHASSIS_RELAX) {
                    robot_ctrl.chassis_mode = CHASSIS_RELAX;
                    // 模式切换为放松时，重置所有标志
                    yaw_align_enable = 0;
                    last_wheel_active = 0;
                    last_qe_active = 0;
                    last_manual_vw = 0.0f;
                } else {
                    robot_ctrl.chassis_mode = CHASSIS_FOLLOW;
                }
            }
            last_toggle_cmd = toggle_cmd;
        }

        /**************************************************************************************************************/

        if (robot_ctrl.monitor.remote_online) {
            if (robot_ctrl.chassis_mode != CHASSIS_RELAX) {
                if (robot_ctrl.chassis_mode == CHASSIS_FOLLOW) {
                    // --- A. 输入源融合 (遥控器摇杆 + 键盘) ---
                    float vx_rc = (abs(rc->vt13.rc_vt13.ch[0]) > RC_DEADZONE) ? rc->vt13.rc_vt13.ch[0] / 660.0f : 0;
                    float vy_rc = (abs(rc->vt13.rc_vt13.ch[1]) > RC_DEADZONE) ? rc->vt13.rc_vt13.ch[1] / 660.0f : 0;
                    float vw_rc = (abs(rc->vt13.rc_vt13.wheel) > RC_DEADZONE) ? rc->vt13.rc_vt13.wheel / 660.0f : 0;

                    float vx_kb = 0, vy_kb = 0, vw_kb = 0;
                    float speed_ratio = (rc->vt13.key_vt13.v & KEY_VT13_SHIFT) ? 1.0f : 0.5f;

                    if (rc->vt13.key_vt13.v & KEY_VT13_W) vy_kb += speed_ratio;
                    if (rc->vt13.key_vt13.v & KEY_VT13_S) vy_kb -= speed_ratio;
                    if (rc->vt13.key_vt13.v & KEY_VT13_A) vx_kb -= speed_ratio;
                    if (rc->vt13.key_vt13.v & KEY_VT13_D) vx_kb += speed_ratio;
                    if (rc->vt13.key_vt13.v & KEY_VT13_Q) vw_kb -= 2.0f * speed_ratio; // 手动左旋
                    if (rc->vt13.key_vt13.v & KEY_VT13_E) vw_kb += 2.0f * speed_ratio; // 手动右旋

                    float total_vx = vx_rc + vx_kb;
                    float total_vy = vy_rc + vy_kb;

                    // --- B. 各向同性限速 ---
                    float v_norm = sqrtf(total_vx * total_vx + total_vy * total_vy);
                    if (v_norm > speed_ratio) {
                        total_vx = total_vx / v_norm * speed_ratio;
                        total_vy = total_vy / v_norm * speed_ratio;
                    }

                    // --- C. 跟随与旋转逻辑（扩展：Q/E+拨轮统一回正）---
                    float yaw_m_pos;
                    yaw_m->get_status(yaw_m, "POS", &yaw_m_pos);
                    float angle_error = Rad_Format(yaw_m_pos - YAW_CENTER_OFFSET);

                    // 步骤1：判断当前拨轮、Q/E是否处于激活状态
                    uint8_t current_wheel_active = (fabsf(vw_rc) > WHEEL_ACTIVE_THRESHOLD) ? 1 : 0;
                    uint8_t current_qe_active = (fabsf(vw_kb) > QE_ACTIVE_THRESHOLD) ? 1 : 0;
                    // 合并手动输入状态（拨轮或Q/E有一个激活，就认为是手动控制阶段）
                    uint8_t current_manual_active = current_wheel_active || current_qe_active;

                    // 步骤2：手动控制阶段，实时保存最后有效旋转速度（统一保存到last_manual_vw）
                    if (current_manual_active) {
                        last_manual_vw = vw_rc + vw_kb; // 保存当前拨轮+Q/E的合成速度（符合原有手动逻辑）
                    }

                    // 步骤3：检测「拨轮」或「Q/E」的松开下降沿，触发回正（二选一触发，避免冲突）
                    uint8_t wheel_release_trigger = (last_wheel_active && !current_wheel_active && !current_qe_active);
                    uint8_t qe_release_trigger = (last_qe_active && !current_qe_active && !current_wheel_active);
                    if ((wheel_release_trigger || qe_release_trigger) && !yaw_align_enable) {
                        yaw_align_enable = 1; // 开启回正使能
                    }

                    // 步骤4：优先级排序：手动控制 > 固定速度回正 > 正常跟随
                    float vw_final = 0;
                    if (current_manual_active) {
                        // 这部分用来加上手动控制情况下的速度的不规则自转
                        // 具体处理逻辑为考虑采用曲线 y = 3t**2 - 2t**3 ,其中t为该阶段进行的进度，映射为（0,1）->(0,1)
                        // 处理方法基本为，得到初始速度与时间后，规定周期，进度相反两方向运动，随后把周期映射到此

                        float raw_manual_vw = vw_rc + vw_kb;
                        const uint32_t period_ms = 1000;
                        float phase = (float)(current_tick % period_ms) / (float)period_ms;
                        float linear_t = 0;
                        if (phase < 0.5f)
                            linear_t = phase * 2.0f;
                        else linear_t = 2.0f - phase * 2.0f;
                        float smooth_t = linear_t * linear_t * (3.0f - 2.0f * linear_t);
                        float chaotic_factor = 0.6f + (0.4f * smooth_t);
                        vw_final = raw_manual_vw * chaotic_factor;

                        // 手动控制阶段：确保关闭自动回正使能，并实时保存带有扰动的最后有效速度
                        // 注意：保存 last_manual_vw 用于后续回正逻辑的起点
                        yaw_align_enable = 0;
                        last_manual_vw = vw_final;

                    } else if (yaw_align_enable) {
                        //该部分用来处理回正逻辑，用来解决回正情况下的猛烈颤抖问题，大致方法仍然为变速曲线，在接近回正点时速度降低，曲线回正变慢
                        //考虑采用绝对位置，即yaw_angle_error / 3.14来作为进度处理，并且考虑曲线y = 1.0 / (1.0 + np.exp(-60.0 * (x - 0.175)))

                        double err_ratio = angle_error / 3.14 ;
                        if (err_ratio > 1.0f) err_ratio = 1.0f;
                        if (err_ratio < 0.03f || fabsf(angle_error) < YAW_ALIGN_THRESHOLD)
                        {
                            yaw_align_enable = 0;
                            vw_final = 0;
                            last_manual_vw = 0;
                        }
                        double curve_factor = 0.8 * (1.0 / (1.0 + exp(-60.0 * (err_ratio - 0.175)))) + 0.2;
                        vw_final = last_manual_vw * curve_factor;

                        // 步骤5：更新上一帧状态记录（供下一帧边缘检测使用）
                        last_wheel_active = current_wheel_active;
                        last_qe_active = current_qe_active;

                        robot_ctrl.chassis.yaw_speed = vw_final * CHASSIS_MAX_RAD;

                        // --- D. 随动坐标系变换 ---
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
            }
            else {
                // 遥控器掉线：红灯快闪
                osDelay(100);
            }

            osDelay(2);
        }
    }
}