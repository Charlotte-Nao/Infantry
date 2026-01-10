//
// Created by 14717 on 2026/1/11.
//

#include "gimbal_task.h"
#include "../Application/robot_global.h"
#include "../../Components/motor/motor.h"
#include "math.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "../../Bsp/uart/bsp_uart.h"
#include "../../Bsp/led/bsp_led.h"
#include "../../Components/remote/remote.h"


#define RC_DEADZONE         10
#define MOUSE_YAW_SENS      0.005f
#define MOUSE_PIT_SENS      0.005f
#define RC_YAW_SENS     0.005f
#define RC_PIT_SENS     0.005f

// 角度归一化
static float Rad_Format(float angle) {
    while (angle >  M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void gimbal_task_func(void const * argument) {
    /* --- 1. 初始化 --- */
    // 调试串口
    struct uart_device* Uart = uart_get_device("uart1_dma");
    Uart->Init(Uart, 115200, 8, 'N', 1);

    // 获取电机设备句柄
    const struct motor_device *yaw_m = motor_get_device("GM6020_YAW");
    const struct motor_device *pit_m = motor_get_device("J4310_PITCH");

    // 获取控制设别句柄
    const RC_ctrl_t *rc = robot_ctrl.rc;

    // 静态状态记录
    static uint8_t last_relax_toggle = 0;  // 用于 Pause/X 切换使能
    static uint8_t last_mode_toggle = 0;   // 用于 E/Custom_L 切换自瞄
    static uint8_t is_initialized = 0;

    float world_yaw_target = 0.0f;
    float world_pit_target = 0.0f;


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
            robot_ctrl.gimbal_mode = GIMBAL_RELAX;
            is_initialized = 0;
        }
        else {
            robot_ctrl.monitor.remote_online = 1;

            /* 输入抽象与边缘检测 */
            // 使能/失能切换 (Pause 或 X)
            uint8_t relax_cmd = (rc->rc.pause) || (rc->key.v & KEY_X);
            uint8_t relax_trigger = (relax_cmd && !last_relax_toggle);

            // 手动/自瞄切换 (Custom_L 或 E)
            uint8_t mode_cmd = (rc->rc.custom_l) || (rc->key.v & KEY_E);
            uint8_t mode_trigger = (mode_cmd && !last_mode_toggle);

            // 状态逻辑处理
            // 处理使能翻转
            if (relax_trigger) {
                if (robot_ctrl.gimbal_mode == GIMBAL_RELAX) {
                    robot_ctrl.gimbal_mode = GIMBAL_REMOTE; // 开启默认进入手动
                } else {
                    robot_ctrl.gimbal_mode = GIMBAL_RELAX;
                    is_initialized = 0; // 关闭时重置初始化标记
                }
            }

            // 在使能状态下处理模式切换
            if (mode_trigger && robot_ctrl.gimbal_mode != GIMBAL_RELAX) {
                robot_ctrl.gimbal_mode = (robot_ctrl.gimbal_mode == GIMBAL_REMOTE) ?
                                          GIMBAL_AUTO : GIMBAL_REMOTE;
            }

            last_relax_toggle = relax_cmd;
            last_mode_toggle = mode_cmd;
        }

        /* 运动控制执行 */
        if (robot_ctrl.monitor.remote_online && robot_ctrl.gimbal_mode != GIMBAL_RELAX) {
            // 首次进入使能状态时，同步当前机械角度，防止云台疯甩
            if (is_initialized == 0) {
                world_yaw_target = robot_ctrl.gimbal.yaw;
                world_pit_target = robot_ctrl.gimbal.pitch;
                is_initialized = 1;
            }

            // 灯光指示：工作模式
            LED_RED_RESET(); LED_BLUE_SET(); LED_GREEN_RESET();


            if (robot_ctrl.gimbal_mode == GIMBAL_REMOTE) {
                // 手动模式：解析遥控器摇杆量
                float ry = (abs(rc->rc.ch[2]) > RC_DEADZONE) ? rc->rc.ch[2] / 660.0f : 0.0f;
                float rx = (abs(rc->rc.ch[3]) > RC_DEADZONE) ? rc->rc.ch[3] / 660.0f : 0.0f;

                world_pit_target += ry * RC_PIT_SENS;
                world_yaw_target -= rx * RC_YAW_SENS;

                // 俯仰角限幅
                if (world_pit_target > 0.45f) world_pit_target = 0.45f;
                if (world_pit_target < -0.40f) world_pit_target = -0.40f;
            } else if (robot_ctrl.gimbal_mode == GIMBAL_AUTO) {
                // 自瞄模式：灯光蓝色提示
                // 此处自瞄解算逻辑...
            }

            float cur_yaw, cur_pit;
            yaw_m->get_status(yaw_m, "POS", &cur_yaw);
            pit_m->get_status(pit_m, "POS", &cur_pit);

            float yaw_out = cur_yaw + Rad_Format(world_yaw_target - robot_ctrl.gimbal.yaw + robot_ctrl.chassis.yaw_speed);
            float pit_out = cur_pit + (world_pit_target - robot_ctrl.gimbal.pitch);

            // 发送控制指令给电机
            yaw_m->set_target(yaw_m, 1, yaw_out);
            pit_m->set_target(pit_m, 1, pit_out);

        }
        else if (robot_ctrl.monitor.remote_online && robot_ctrl.gimbal_mode == GIMBAL_RELAX) {
            //已连接但失能：红灯常亮
            LED_GREEN_RESET(); LED_BLUE_RESET(); LED_RED_SET();
        }


        osDelay(2);
    }
}

