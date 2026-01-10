#include "../Application/chassis_kinematics.h"
#include "../ALL_Task/logic_task.h"
#include "../Components/motor/motor.h"
#include "../Bsp/uart/bsp_uart.h"
#include "../Application/robot_global.h"
#include "math.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "stdio.h"

/* --- 逻辑常量与控制参数 --- */
#define GIMBAL_YAW_SENS     0.010f
#define GIMBAL_PIT_SENS     0.002f
#define MOUSE_YAW_SENS      0.0004f
#define MOUSE_PIT_SENS      0.0002f
#define FOLLOW_P_GAIN       0.5f
#define RC_DEADZONE         10
#define CHASSIS_ACCEL       0.005f
#define KEYBOARD_V_BASE     0.6f

#define FRIC_SPEED_ON       8000.0f
#define STIR_V_FIRE         -5000.0f   // 正转射击
#define YAW_CENTER_OFFSET   0.0f

/* --- 静态控制变量 --- */
static float world_yaw_target = 0.0f;
static float world_pit_target = 0.0f;
static float vx_ramp = 0.0f, vy_ramp = 0.0f;
static uint8_t is_initialized = 0;
static uint32_t last_rc_tick = 0;

static uint8_t last_f_key = 0; // 摩擦轮切换
static uint8_t last_g_key = 0; // 底盘模式切换

static float Rad_Format(float angle) {
    while (angle >  M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void logic_task_func(void const * argument) {
    float wheel_targets[4] = {0};

    const struct motor_device *yaw_m      = motor_get_device("GM6020_YAW");
    const struct motor_device *pit_m      = motor_get_device("J4310_PITCH");
    const struct motor_device *shoot_L_m  = motor_get_device("M3508_SHOOT_L");
    const struct motor_device *shoot_R_m  = motor_get_device("M3508_SHOOT_R");
    const struct motor_device *trigger_m  = motor_get_device("M2006_TRIGGER");
    struct motor_device *chassis[4];
    for(int i=0; i<4; i++) {
        char name[25]; sprintf(name, "M3508_CHASSIS_%d", i+1);
        chassis[i] = motor_get_device(name);
    }

    while (1) {
        uint32_t current_tick = osKernelSysTick();

        if (robot_ctrl.rc != NULL) {
            last_rc_tick = current_tick;

            /* --- A1. G键 切换底盘模式 (RELAX -> STATIC -> FOLLOW) --- */
            uint8_t current_g_key = (robot_ctrl.rc->key.v >> 10) & 0x01; // bit10: G
            if (current_g_key && !last_g_key) {
                if (robot_ctrl.chassis_mode == CHASSIS_RELAX)       robot_ctrl.chassis_mode = CHASSIS_STATIC;
                else if (robot_ctrl.chassis_mode == CHASSIS_STATIC) robot_ctrl.chassis_mode = CHASSIS_FOLLOW;
                else                                                robot_ctrl.chassis_mode = CHASSIS_RELAX;
            }
            last_g_key = current_g_key;

            // 云台随动底盘失能状态
            robot_ctrl.gimbal_mode = (robot_ctrl.chassis_mode == CHASSIS_RELAX) ? GIMBAL_RELAX : GIMBAL_REMOTE;

            /* --- A2. F键 切换摩擦轮 --- */
            uint8_t current_f_key = (robot_ctrl.rc->key.v >> 9) & 0x01; // bit9: F
            if (current_f_key && !last_f_key) {
                robot_ctrl.shoot_mode = (robot_ctrl.shoot_mode == SHOOT_STOP) ? SHOOT_READY : SHOOT_STOP;
            }
            last_f_key = current_f_key;
        }

        // 掉线保护
        if (current_tick - last_rc_tick > 200) {
            robot_ctrl.chassis_mode = CHASSIS_RELAX;
            robot_ctrl.gimbal_mode = GIMBAL_RELAX;
            robot_ctrl.shoot_mode = SHOOT_STOP;
        }

        /* --- B. 底盘控制 --- */
        if (robot_ctrl.chassis_mode != CHASSIS_RELAX) {
            float vx_target = 0, vy_target = 0, vw_target = 0;
            float k_speed = (robot_ctrl.rc->key.v & (1 << 4)) ? 1.0f : KEYBOARD_V_BASE;

            if (robot_ctrl.rc->key.v & (1 << 0)) vx_target += k_speed; // W
            if (robot_ctrl.rc->key.v & (1 << 1)) vx_target -= k_speed; // S
            if (robot_ctrl.rc->key.v & (1 << 2)) vy_target -= k_speed; // A
            if (robot_ctrl.rc->key.v & (1 << 3)) vy_target += k_speed; // D

            if (vx_target == 0 && vy_target == 0) {
                vx_target = (abs(robot_ctrl.rc->rc.ch[1]) > RC_DEADZONE) ? robot_ctrl.rc->rc.ch[1] / 660.0f : 0;
                vy_target = (abs(robot_ctrl.rc->rc.ch[0]) > RC_DEADZONE) ? -robot_ctrl.rc->rc.ch[0] / 660.0f : 0;
            }

            if (vx_ramp < vx_target) vx_ramp += CHASSIS_ACCEL;
            else if (vx_ramp > vx_target) vx_ramp -= CHASSIS_ACCEL;
            if (vy_ramp < vy_target) vy_ramp += CHASSIS_ACCEL;
            else if (vy_ramp > vy_target) vy_ramp -= CHASSIS_ACCEL;

            if (robot_ctrl.chassis_mode == CHASSIS_FOLLOW && yaw_m) {
                float cur_yaw_rad; yaw_m->get_status(yaw_m, "POS", &cur_yaw_rad);
                vw_target = -Rad_Format(cur_yaw_rad - YAW_CENTER_OFFSET) * FOLLOW_P_GAIN;
            }

            Chassis_Omni_Inverse_Kinematics(vx_ramp, vy_ramp, vw_target, wheel_targets);
            for(int i=0; i<4; i++) if(chassis[i]) chassis[i]->set_target(chassis[i], 1, wheel_targets[i]);
        } else {
            vx_ramp = 0; vy_ramp = 0;
            for(int i=0; i<4; i++) if(chassis[i]) chassis[i]->set_target(chassis[i], 1, 0);
        }

        /* --- C. 云台控制 (直接响应) --- */
        if (robot_ctrl.gimbal_mode != GIMBAL_RELAX && yaw_m && pit_m) {
            if (!is_initialized) {
                world_yaw_target = robot_ctrl.imu.yaw;
                world_pit_target = robot_ctrl.imu.pitch;
                is_initialized = 1;
            }

            world_yaw_target -= robot_ctrl.rc->mouse.x * MOUSE_YAW_SENS;
            world_pit_target += robot_ctrl.rc->mouse.y * MOUSE_PIT_SENS;

            float rx = (abs(robot_ctrl.rc->rc.ch[3]) > RC_DEADZONE) ? robot_ctrl.rc->rc.ch[3] / 660.0f : 0;
            float ry = (abs(robot_ctrl.rc->rc.ch[2]) > RC_DEADZONE) ? robot_ctrl.rc->rc.ch[2] / 660.0f : 0;
            world_yaw_target -= rx * GIMBAL_YAW_SENS;
            world_pit_target += ry * GIMBAL_PIT_SENS;

            if (world_pit_target > 0.45f)  world_pit_target = 0.45f;
            if (world_pit_target < -0.40f) world_pit_target = -0.40f;

            float cur_yaw, cur_pit;
            yaw_m->get_status(yaw_m, "POS", &cur_yaw);
            pit_m->get_status(pit_m, "POS", &cur_pit);

            float yaw_out = cur_yaw + Rad_Format(world_yaw_target - robot_ctrl.imu.yaw) * 1.3f;
            float pit_out = cur_pit + (world_pit_target - robot_ctrl.imu.pitch) * 1.2f;

            yaw_m->set_target(yaw_m, 1, yaw_out);
            pit_m->set_target(pit_m, 1, pit_out);
        } else {
            is_initialized = 0;
            if(yaw_m) yaw_m->set_target(yaw_m, 1, 0);
            if(pit_m) pit_m->set_target(pit_m, 1, 0);
        }

        /* --- D. 发射逻辑 (包含中键反转) --- */
        if (robot_ctrl.shoot_mode != SHOOT_STOP) {
            if(shoot_L_m) shoot_L_m->set_target(shoot_L_m, 1,  FRIC_SPEED_ON);
            if(shoot_R_m) shoot_R_m->set_target(shoot_R_m, 1, -FRIC_SPEED_ON);

            if (robot_ctrl.rc->mouse.press_m) { // 中键反转优先级最高
                if(trigger_m) trigger_m->set_target(trigger_m, 1, -STIR_V_FIRE);
            } else if (robot_ctrl.rc->rc.trigger || robot_ctrl.rc->mouse.press_l) {
                if(trigger_m) trigger_m->set_target(trigger_m, 1, STIR_V_FIRE);
            } else {
                if(trigger_m) trigger_m->set_target(trigger_m, 1, 0);
            }
        } else {
            if(shoot_L_m) shoot_L_m->set_target(shoot_L_m, 1, 0);
            if(shoot_R_m) shoot_R_m->set_target(shoot_R_m, 1, 0);
            if(trigger_m) trigger_m->set_target(trigger_m, 1, 0);
        }

        osDelay(2);
    }
}