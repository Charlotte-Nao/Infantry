#include "../Application/chassis_kinematics.h"
#include "../ALL_Task/logic_task.h"
#include <stdio.h>
#include "../Components/motor/motor.h"
#include "../Bsp/uart/bsp_uart.h"
#include "../Application/robot_global.h"
#include "math.h"
#include "stdlib.h"
#include "cmsis_os.h"

/* --- 逻辑常量定义 --- */
#define GIMBAL_YAW_SENS    0.020f
#define GIMBAL_PIT_SENS    0.010f
#define FOLLOW_P_GAIN      0.5f
#define GIMBAL_IMU_KP      1.2f
#define GIMBAL_YAW_FF      1.2f      // 云台对底盘自旋的前馈补偿
#define CHASSIS_YAW_LAG    30.0f     // 小陀螺平移相位补偿
#define SPIN_SPEED         1.0f      // 小陀螺自旋目标转速 (rad/s)
#define RC_DEADZONE        20

#define FRIC_SPEED_ON      500.0f//6500.0f   // 摩擦轮转速
#define STIR_V_FIRE        -8000.0f     // 拨弹轮发射速度
#define STIR_V_REVERSE     2000.0f     // 拨弹轮反转速度
#define STIR_MAX_CURRENT   10000.0f  // 拨弹轮电流阈值 (防卡弹)

#define YAW_CENTER_RAD_OFFSET  3.24f

/* --- 遥控器通道定义 --- */
#define RC_SW_R             robot_ctrl.rc->rc.s[0]
#define RC_SW_L             robot_ctrl.rc->rc.s[1]
#define RC_CH_R_X           robot_ctrl.rc->rc.ch[0]
#define RC_CH_R_Y           robot_ctrl.rc->rc.ch[1]
#define RC_CH_L_X           robot_ctrl.rc->rc.ch[2]
#define RC_CH_L_Y           robot_ctrl.rc->rc.ch[3]
#define RC_CH_T           robot_ctrl.rc->rc.ch[4]


/*调试串口*/
struct uart_device *Uart;
char uart_buf[256];

static float world_yaw_target = 0.0f;
static float world_pit_target = 0.0f;
static uint8_t is_initialized = 0;  //防甩头设计

static float Rad_Format(float angle) {
    while (angle >  M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void logic_task_func(void const * argument) {
    float wheel_targets[4] = {0};

    //获取电机句柄
    const struct motor_device *yaw_m   = motor_get_device("GM6020_YAW");
    const struct motor_device *pit_m   = motor_get_device("J4310_PITCH");
    const struct motor_device *shoot_L_m  = motor_get_device("M3508_SHOOT_L");
    const struct motor_device *shoot_R_m  = motor_get_device("M3508_SHOOT_R");
    const struct motor_device *trigger_m  = motor_get_device("M2006_TRIGGER");
    struct motor_device *chassis[4];
    for(int i=0; i<4; i++) {
        char name[25]; sprintf(name, "M3508_CHASSIS_%d", i+1);
        chassis[i] = motor_get_device(name);
    }

    //获取串口句柄用于调试
    Uart = uart_get_device("uart1_dma");
    Uart->Init(Uart, 115200, 8, 'N', 1);

    //开机等待遥控器信号
    // while (!(switch_is_down(RC_SW_R)&switch_is_down(RC_SW_L))) {
    //     osDelay(100);
    // }

    while (1) {
        // 遥控器离线检查 (待完善)
        if (robot_ctrl.rc == NULL) {
            osDelay(2); continue;
        }

        /* --- 底盘模式切换 --- */
        // 右开关 下：失能模式 中：独立模式 上：跟随模式
        if (switch_is_down(RC_SW_R)) {
            robot_ctrl.chassis_mode = CHASSIS_RELAX;
        }
        else if (switch_is_mid(RC_SW_R)) {
            robot_ctrl.chassis_mode = CHASSIS_STATIC;
        }
        else if (switch_is_up(RC_SW_R)) {
            robot_ctrl.chassis_mode = CHASSIS_FOLLOW;
        }

        /* --- 云台模式切换 --- */
        // 左开关 下：失能模式 中：遥控模式 上：自瞄模式
        if (switch_is_down(RC_SW_L)) {
            robot_ctrl.gimbal_mode = GIMBAL_RELAX;
            is_initialized = 0;
        }
        else if (switch_is_mid(RC_SW_L)) {
            robot_ctrl.gimbal_mode = GIMBAL_REMOTE;
        }
        else if (switch_is_up(RC_SW_L)) {
            robot_ctrl.gimbal_mode = GIMBAL_AUTO;
        }

        /* --- C. 底盘运动控制 --- */
        if (robot_ctrl.chassis_mode == CHASSIS_STATIC && yaw_m) {
            float cur_yaw_rad; yaw_m->get_status(yaw_m, "POS", &cur_yaw_rad);
            const float rel_yaw = Rad_Format(cur_yaw_rad - YAW_CENTER_RAD_OFFSET);
            const float lx = (abs(robot_ctrl.rc->rc.ch[0]) > RC_DEADZONE) ? robot_ctrl.rc->rc.ch[0] / 660.0f : 0;
            const float ly = (abs(robot_ctrl.rc->rc.ch[1]) > RC_DEADZONE) ? robot_ctrl.rc->rc.ch[1] / 660.0f : 0;

            float vx = 0, vy = 0, vw = 0;

            if (robot_ctrl.chassis_mode != CHASSIS_STATIC) {
                float yaw_for_matrix = rel_yaw + vw * CHASSIS_YAW_LAG;
                float cos_y = cosf(yaw_for_matrix);
                float sin_y = sinf(yaw_for_matrix);
                vx = ly * cos_y - lx * sin_y;
                vy = ly * sin_y + lx * cos_y;
            } else { vx = ly; vy = 0; }

            Chassis_Omni_Inverse_Kinematics(vx, vy, vw, wheel_targets);


            for(int i=0; i<4; i++) {
                if(chassis[i]) chassis[i]->set_target(chassis[i], 1, wheel_targets[i]);
            }
        }
        else {
            for(int i=0; i<4; i++) {
                if(chassis[i]) chassis[i]->set_target(chassis[i], 1, 0);
            }
        }


        /* --- 云台锁头逻辑 --- */
        if (robot_ctrl.gimbal_mode != GIMBAL_RELAX && yaw_m && pit_m) {
            if (!is_initialized) {
                world_yaw_target = robot_ctrl.imu.yaw;
                world_pit_target = robot_ctrl.imu.pitch;
                is_initialized = 1;
            }

            float ry_x = (abs(robot_ctrl.rc->rc.ch[2]) > RC_DEADZONE) ? robot_ctrl.rc->rc.ch[2] / 660.0f : 0;
            float ry_y = (abs(robot_ctrl.rc->rc.ch[3]) > RC_DEADZONE) ? robot_ctrl.rc->rc.ch[3] / 660.0f : 0;
            world_yaw_target = Rad_Format(world_yaw_target - ry_x * GIMBAL_YAW_SENS);
            world_pit_target += ry_y * GIMBAL_PIT_SENS;

            // Pitch 限幅
            if (world_pit_target > 0.45f) world_pit_target = 0.45f;
            if (world_pit_target < -0.4f) world_pit_target = -0.4f;

            float cur_yaw_rad, cur_pit_rad;
            yaw_m->get_status(yaw_m, "POS", &cur_yaw_rad);
            pit_m->get_status(pit_m, "POS", &cur_pit_rad);

            Uart->Print(Uart, "yaw: %f\r\n", cur_yaw_rad);

            // 自旋速度计算
            if (robot_ctrl.chassis_mode == CHASSIS_SPIN) vw_planned = SPIN_SPEED;
            else if (robot_ctrl.chassis_mode == CHASSIS_FOLLOW) vw_planned = -Rad_Format(cur_yaw_rad - YAW_CENTER_RAD_OFFSET) * FOLLOW_P_GAIN;
            else if (robot_ctrl.chassis_mode == CHASSIS_STATIC) vw_planned = (abs(robot_ctrl.rc->rc.ch[0]) > RC_DEADZONE ? robot_ctrl.rc->rc.ch[0] / 660.0f : 0) * 3.0f;

            // 串级 PID 目标下发
            float yaw_target = cur_yaw_rad + Rad_Format(world_yaw_target - robot_ctrl.imu.yaw) * GIMBAL_IMU_KP + vw_planned * GIMBAL_YAW_FF;
            float pit_target = cur_pit_rad + (world_pit_target - robot_ctrl.imu.pitch) * GIMBAL_IMU_KP;

            yaw_m->set_target(yaw_m, 1, yaw_target);
            pit_m->set_target(pit_m, 1, pit_target);
        }



        /* --- D. 发射控制系统 --- */
        if (robot_ctrl.gimbal_mode != GIMBAL_RELAX) {
            // 左开关 s[1] 不在底部(2)时开启摩擦轮
            if (robot_ctrl.rc->rc.s[1] != 2 && robot_ctrl.rc->rc.s[1] != 0) {
                robot_ctrl.shoot_mode = SHOOT_FIRE_CONTINUE;
                // if(fric_l) fric_l->set_target(fric_l, 1, 0);
                // if(fric_r) fric_r->set_target(fric_r, 1, 0);
                if(shoot_L_m) shoot_L_m->set_target(shoot_L_m, 1,  FRIC_SPEED_ON);
                if(shoot_R_m) shoot_R_m->set_target(shoot_R_m, 1, -FRIC_SPEED_ON);
            } else {
                robot_ctrl.shoot_mode = SHOOT_STOP;
                if(shoot_L_m) shoot_L_m->set_target(shoot_L_m, 1, 0);
                if(shoot_R_m) shoot_R_m->set_target(shoot_R_m, 1, 0);
            }

            // 拨弹逻辑
            float stir_v_target = 0;
            if (robot_ctrl.shoot_mode != SHOOT_STOP) {
                if (robot_ctrl.rc->rc.ch[4] > 400) stir_v_target = STIR_V_FIRE;
                else if (robot_ctrl.rc->rc.ch[4] < -400) stir_v_target = STIR_V_REVERSE;

                // 使用 CURRENT 参数进行防卡弹
                if(trigger_m) {
                    float stir_curr = 0;
                    trigger_m->get_status(trigger_m, "CURRENT", &stir_curr);
                    if (fabs(stir_curr) > STIR_MAX_CURRENT && stir_v_target > 0) {
                        stir_v_target = STIR_V_REVERSE;
                    }
                }
            }
            if(trigger_m) trigger_m->set_target(trigger_m, 1, stir_v_target);
        } else {
            robot_ctrl.shoot_mode = SHOOT_STOP;
            if(shoot_L_m) shoot_L_m->set_target(shoot_L_m, 1, 0);
            if(shoot_R_m) shoot_R_m->set_target(shoot_R_m, 1, 0);
            if(trigger_m) trigger_m->set_target(trigger_m, 1, 0);
        }
        osDelay(2);
    }
}