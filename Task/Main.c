//
// Created by 14717 on 2025/8/27.
//

#include "Main.h"
#include "cmsis_os.h"
#include <math.h>
#include "Auto.h"
#include "Auto_Aim.h"

/**********************************************************************************************************************/
/*应用INCLUDE*/
#include "../Application/motor_control.h"
#include "../Application/auto_aim.h"

/**********************************************************************************************************************/
/*设备INCLUDE*/
#include "../Device/motor/motor.h"

/**********************************************************************************************************************/
/*驱动INCLUDE*/
#include "../Driver/LED/LED.h"
#include "../Driver/can/can.h"
#include "../Driver/uart/dvc_uart.h"
#include "../Driver/remote/remote.h"
#include "../Driver/motor_pwm/fire_control.h"

/**********************************************************************************************************************/
/*宏定义*/

/*速度*/
#define YAW_SPEED ((fp32)pi / 576)
#define PITCH_SPEED ((fp32)pi / 576)
#define CHASSIS_SPEED ((fp32)pi / 216)

/*重力补偿系数*/
#define GRAVITY_COMPENSATION_COEFFICIENT ((uint16_t)2500)

/*遥控器摇杆宏定义*/
/*通道值 -660~660 */
#define RC_CH_VALUE_MIN         ((uint16_t)0)
#define RC_CH_VALUE_OFFSET      ((uint16_t)50)
#define RC_CH_VALUE_MAX         ((uint16_t)660)

/*遥控器拨杆宏定义*/
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

/*科学常量宏定义*/
#define pi (fp32)M_PI

/*电机ID（对应在motor_list中的索引）*/
typedef enum
{
    CHASSIS_0 = 0,
    CHASSIS_1 = 1,
    CHASSIS_2 = 2,
    CHASSIS_3 = 3,
    YAW_M = 4,
    PITCH_M = 5,
    SHOOT_M = 6,
} motor;

/*通道设置*/
typedef enum
{
    /*
     * 0 ---> 右手水平摇杆    -660~660
     * 1 ---> 右手竖直摇杆    -660~660
     * 2 ---> 左手水平摇杆    -660~660
     * 3 ---> 左手竖直摇杆    -660~660
     * 4 ---> 左上拨轮       -660~660
     */
    X = 0,
    Y = 1,
    YAW = 2,
    PITCH = 3,
    SHOOT_RATE = 4,
} channel;
typedef enum
{
    R = 0,
    L = 1,
} Switch;

/**********************************************************************************************************************/
/*全局变量定义*/
const RC_ctrl_t *RC_msg;
struct uart_device *Uart;

/**********************************************************************************************************************/
/*底盘控制参数*/
int16_t motor_current_1;
int16_t motor_current_2;
int16_t motor_current_3;
int16_t motor_current_4;

/**********************************************************************************************************************/
/*云台控制参数*/
fp32 yaw_speed;
fp32 yaw_pose;
fp32 last_INS_angle_yaw;
int16_t yaw_voltage;
struct PID *yaw_keep_pid;
struct PID *yaw_auto_pid;
fp32 yaw_aim;
fp32 last_yaw_aim;

fp32 pitch_speed;
fp32 pitch_pose;
fp32 last_INS_angle_pitch;
int16_t pitch_voltage;
int16_t pitch_g_voltage;
struct PID *pitch_keep_pid;
struct PID *pitch_auto_pid;
fp32 pitch_aim;
fp32 last_pitch_aim;

bool_t target_trigger;
bool_t target_sw_trigger;

int16_t shoot_speed_pwm;

int16_t shoot_rate_current;
fp32 shoot_self_help;

/*--------------------------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------------------------*/

void main_task(void const *argument) {

    /******************************************************************************************************************/
    /*等待设备初始化*/
    vTaskDelay(4000);

    /******************************************************************************************************************/
    /*软件初始化*/
    motor_device_list_init();
    can_filter_init();
    RC_init();
    RC_msg = RC_get_handle();
    vTaskDelay(100);
    Uart = uart_get_device("uart1_dma");
    Uart->Init(Uart, 115200, 8, 'N', 1);

    /******************************************************************************************************************/
    /*发射机构启动保险*/

    //while (!(switch_is_up(RC_msg->rc.s[L]) && switch_is_up(RC_msg->rc.s[R]))){}
    fire_init();
    vTaskDelay(1000);

    /******************************************************************************************************************/
    /*YAW轴初始化*/

    motor_device_list[YAW_M]->update(motor_device_list[YAW_M]);
    yaw_pose = INS_angle[0];
    last_INS_angle_yaw = INS_angle[0];
    yaw_keep_pid = pid_get_device("yaw_keep");
    const fp32 yaw_keep_pid_param[3] = {0.2f, 0, -10};
    yaw_keep_pid->init(yaw_keep_pid, yaw_keep_pid_param, pi / 18, 0);
    yaw_auto_pid = pid_get_device("yaw_auto");
    const fp32 yaw_auto_pid_param[3] = {0.05f, 0, -0.1f};
    yaw_auto_pid->init(yaw_auto_pid, yaw_auto_pid_param, pi / 18, 0);

    /******************************************************************************************************************/
    /*PITCH轴初始化*/

    motor_device_list[PITCH_M]->update(motor_device_list[PITCH_M]);
    pitch_pose = INS_angle[2];
    last_INS_angle_pitch = INS_angle[2];
    pitch_keep_pid = pid_get_device("pitch_keep");
    const fp32 pitch_keep_pid_param[3] = {0.05f, 0, -2.5f};
    pitch_keep_pid->init(pitch_keep_pid, pitch_keep_pid_param, pi / 18, 0);
    pitch_auto_pid = pid_get_device("pitch_auto");
    const fp32 pitch_auto_pid_param[3] = {0.025f, 0, -0.5f};
    pitch_auto_pid->init(pitch_auto_pid, pitch_auto_pid_param, pi / 18, 0);


    while (1) {

        /***********************************************************************************************************/
        /*自瞄*/

        //Uart->Print(Uart, "%d%d\n", (int)(1000*yaw_pose), (int)(1000*pitch_pose));
        if (target_ok == 1) {
            LED_GREEN_SET();
        }
        else {
            LED_GREEN_RESET();
        }
        if (RC_msg->rc.ch[PITCH] == -RC_CH_VALUE_MAX) {//开自瞄
            if (target_ok == 1) {
                auto_aim_control(&target_info, &yaw_aim, &pitch_aim);
                yaw_pose = yaw_aim;
                pitch_pose = pitch_aim;
                Uart->Print(Uart, "%d,%d\n", (int)(1000*INS_angle[0]), (int)(1000*INS_angle[2]));
            }

            yaw_speed = yaw_keep_pid->calc(yaw_keep_pid, INS_angle[0], yaw_pose, INS_angle[0] - last_INS_angle_yaw);
            last_INS_angle_yaw = INS_angle[0];
            motor_device_list[YAW_M]->set_speed(motor_device_list[YAW_M], yaw_speed);
            motor_device_list[YAW_M]->update(motor_device_list[YAW_M]);
            yaw_voltage = motor_device_list[YAW_M]->given_voltage(motor_device_list[YAW_M]);

            pitch_speed = pitch_keep_pid->calc(pitch_keep_pid, INS_angle[2], pitch_pose, INS_angle[2]-last_INS_angle_pitch);
            last_INS_angle_pitch = INS_angle[2];
            motor_device_list[PITCH_M]->set_speed(motor_device_list[PITCH_M], -pitch_speed);
            motor_device_list[PITCH_M]->update(motor_device_list[PITCH_M]);
            pitch_voltage = motor_device_list[PITCH_M]->given_voltage(motor_device_list[PITCH_M]);
        //     if (target_ok == 1) {
        //         auto_aim_control(&target_info, &yaw_aim, &pitch_aim);
        //
        //         yaw_speed = yaw_auto_pid->calc(yaw_auto_pid, yaw_aim, 0, yaw_aim - last_yaw_aim);
        //         last_yaw_aim = yaw_aim;
        //
        //         pitch_speed = pitch_auto_pid->calc(pitch_auto_pid, pitch_aim, 0, pitch_aim - last_pitch_aim);
        //         last_pitch_aim = pitch_aim;
        //
        //         target_trigger = 1;
        //     } else if (target_trigger == 1) {
        //         target_trigger = 0;
        //         yaw_pose = INS_angle[0];
        //         pitch_pose = INS_angle[2];
        //     } else {
        //         yaw_aim = 0;
        //         pitch_aim = 0;
        //         yaw_speed = yaw_keep_pid->calc(yaw_keep_pid, INS_angle[0], yaw_pose, INS_angle[0] - last_INS_angle_yaw);
        //         last_INS_angle_yaw = INS_angle[0];
        //
        //         pitch_speed = pitch_keep_pid->calc(pitch_keep_pid, INS_angle[2], pitch_pose,
        //                                            INS_angle[2] - last_INS_angle_pitch);
        //         last_INS_angle_pitch = INS_angle[2];
        //     }
        //
        //     motor_device_list[YAW_M]->set_speed(motor_device_list[YAW_M], yaw_speed);
        //     motor_device_list[YAW_M]->update(motor_device_list[YAW_M]);
        //     yaw_voltage = motor_device_list[YAW_M]->given_voltage(motor_device_list[YAW_M]);
        //
        //     motor_device_list[PITCH_M]->set_speed(motor_device_list[PITCH_M], -pitch_speed);
        //     motor_device_list[PITCH_M]->update(motor_device_list[PITCH_M]);
        //     pitch_voltage = motor_device_list[PITCH_M]->given_voltage(motor_device_list[PITCH_M]);
        //
        //     target_sw_trigger = 1;
        // } else if (target_sw_trigger == 1) {
        //     //防止切换模式甩头
        //     target_sw_trigger = 0;
        //     yaw_pose = INS_angle[0];
        //     pitch_pose = INS_angle[2];
        }
        else{
            /***********************************************************************************************************/
            /*YAW轴控制*/
            /*线性控制速度*/
            if (!switch_is_up(RC_msg->rc.s[R])) {
                if (RC_msg->rc.ch[YAW] > RC_CH_VALUE_OFFSET || RC_msg->rc.ch[YAW] < -RC_CH_VALUE_OFFSET) {
                    yaw_pose -= (float) (RC_msg->rc.ch[YAW]) / 660 * YAW_SPEED;
                }
            }
            yaw_speed = yaw_keep_pid->calc(yaw_keep_pid, INS_angle[0], yaw_pose, INS_angle[0] - last_INS_angle_yaw);
            last_INS_angle_yaw = INS_angle[0];
            motor_device_list[YAW_M]->set_speed(motor_device_list[YAW_M], yaw_speed);
            motor_device_list[YAW_M]->update(motor_device_list[YAW_M]);
            yaw_voltage = motor_device_list[YAW_M]->given_voltage(motor_device_list[YAW_M]);


            /***********************************************************************************************************/
            /*PITCH轴控制*/
            /*线性控制速度*/
            if (!switch_is_up(RC_msg->rc.s[R])) {
                if ((RC_msg->rc.ch[PITCH] > RC_CH_VALUE_OFFSET && pitch_pose > -0.45) || (
                        RC_msg->rc.ch[PITCH] < -RC_CH_VALUE_OFFSET && pitch_pose < 0.45)) {
                    pitch_pose -= (float) (RC_msg->rc.ch[PITCH]) / 660 * PITCH_SPEED;
                }
            }
            pitch_speed = pitch_keep_pid->calc(pitch_keep_pid, INS_angle[2], pitch_pose, INS_angle[2]-last_INS_angle_pitch);
            last_INS_angle_pitch = INS_angle[2];
            motor_device_list[PITCH_M]->set_speed(motor_device_list[PITCH_M], -pitch_speed);
            motor_device_list[PITCH_M]->update(motor_device_list[PITCH_M]);
            pitch_voltage = motor_device_list[PITCH_M]->given_voltage(motor_device_list[PITCH_M]);
        }

        /***************************************************************************************************************/

        /*PITCH重力补偿*/
        pitch_g_voltage = (uint16_t)(cos(INS_angle[2])*GRAVITY_COMPENSATION_COEFFICIENT);

        /***************************************************************************************************************/
        /*射速控制*/
        /*PWM控制速度*/
        if (switch_is_up(RC_msg->rc.s[1])) {
            shoot_speed_pwm = 1000;
        }
        else if (switch_is_mid(RC_msg->rc.s[1])) {
            shoot_speed_pwm = 1200;
        }
        else if (switch_is_down(RC_msg->rc.s[1])) {
            shoot_speed_pwm = 1450;
        }

        /***************************************************************************************************************/
        /*射频控制及卡弹自救*/
        if (!switch_is_up(RC_msg->rc.s[L])) {
            if (RC_msg->rc.ch[SHOOT_RATE] > RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET) {
                shoot_rate_current = motor_turn_speed(motor_device_list[SHOOT_M], pi / 200);

            }
            else if (RC_msg->rc.ch[SHOOT_RATE] > RC_CH_VALUE_OFFSET) {
                shoot_rate_current = motor_turn_speed(motor_device_list[SHOOT_M], pi / 1600);
            }
            else if (RC_msg->rc.ch[SHOOT_RATE] < -RC_CH_VALUE_OFFSET){/*卡弹反转*/
                shoot_rate_current = motor_turn_speed(motor_device_list[SHOOT_M], -pi / 3200);
            }
            else {
                shoot_rate_current = motor_turn_speed(motor_device_list[SHOOT_M], 0);
            }
        }
        else {
            shoot_rate_current = motor_turn_speed(motor_device_list[SHOOT_M], 0);
        }

        /**************************************************************************************************************/
        /*底盘控制*/
        if (switch_is_mid(RC_msg->rc.s[R])) {
            if (RC_msg->rc.ch[Y] > RC_CH_VALUE_OFFSET || RC_msg->rc.ch[Y] < -RC_CH_VALUE_OFFSET || RC_msg->rc.ch[X] > RC_CH_VALUE_OFFSET || RC_msg->rc.ch[X] < -RC_CH_VALUE_OFFSET) {
                motor_current_1 = motor_turn_speed(motor_device_list[CHASSIS_0], (float)(RC_msg->rc.ch[Y] + RC_msg->rc.ch[X])/660 * CHASSIS_SPEED);;
                motor_current_2 = motor_turn_speed(motor_device_list[CHASSIS_1], (float)(-RC_msg->rc.ch[Y] - RC_msg->rc.ch[X])/660 * CHASSIS_SPEED);;
                motor_current_3 = motor_turn_speed(motor_device_list[CHASSIS_2], (float)(-RC_msg->rc.ch[Y] + RC_msg->rc.ch[X])/660 * CHASSIS_SPEED);;
                motor_current_4 = motor_turn_speed(motor_device_list[CHASSIS_3], (float)(RC_msg->rc.ch[Y] - RC_msg->rc.ch[X])/660 * CHASSIS_SPEED);;
            }
            else {
                motor_current_1 = motor_turn_speed(motor_device_list[CHASSIS_0], 0);;
                motor_current_2 = motor_turn_speed(motor_device_list[CHASSIS_1], 0);;
                motor_current_3 = motor_turn_speed(motor_device_list[CHASSIS_2], 0);;
                motor_current_4 = motor_turn_speed(motor_device_list[CHASSIS_3], 0);;
            }
        }

        if (switch_is_down(RC_msg->rc.s[R])) {
            if (RC_msg->rc.ch[Y] > RC_CH_VALUE_OFFSET || RC_msg->rc.ch[Y] < -RC_CH_VALUE_OFFSET || RC_msg->rc.ch[X] > RC_CH_VALUE_OFFSET || RC_msg->rc.ch[X] < -RC_CH_VALUE_OFFSET) {
                if (RC_msg->rc.ch[Y] > RC_CH_VALUE_OFFSET) {
                    motor_current_1 = motor_turn_speed(motor_device_list[CHASSIS_0],
                                                 (float) (RC_msg->rc.ch[Y] + RC_msg->rc.ch[X]) / 660 * CHASSIS_SPEED);
                    motor_current_2 = motor_turn_speed(motor_device_list[CHASSIS_1],
                                                 (float) (-RC_msg->rc.ch[Y] + RC_msg->rc.ch[X]) / 660 * CHASSIS_SPEED);
                    motor_current_3 = motor_turn_speed(motor_device_list[CHASSIS_2],
                                                 (float) (-RC_msg->rc.ch[Y] + RC_msg->rc.ch[X]) / 660 * CHASSIS_SPEED);
                    motor_current_4 = motor_turn_speed(motor_device_list[CHASSIS_3],
                                                 (float) (RC_msg->rc.ch[Y] + RC_msg->rc.ch[X]) / 660 * CHASSIS_SPEED);
                }
                else if (RC_msg->rc.ch[Y] < -3*RC_CH_VALUE_OFFSET) {
                    motor_current_1 = motor_turn_speed(motor_device_list[CHASSIS_0],
                                                 (float) (RC_msg->rc.ch[Y] - RC_msg->rc.ch[X]) / 660 * CHASSIS_SPEED);
                    motor_current_2 = motor_turn_speed(motor_device_list[CHASSIS_1],
                                                 (float) (-RC_msg->rc.ch[Y] - RC_msg->rc.ch[X]) / 660 * CHASSIS_SPEED);
                    motor_current_3 = motor_turn_speed(motor_device_list[CHASSIS_2],
                                                 (float) (-RC_msg->rc.ch[Y] - RC_msg->rc.ch[X]) / 660 * CHASSIS_SPEED);
                    motor_current_4 = motor_turn_speed(motor_device_list[CHASSIS_3],
                                                 (float) (RC_msg->rc.ch[Y] - RC_msg->rc.ch[X]) / 660 * CHASSIS_SPEED);
                }
                else if (RC_msg->rc.ch[Y] < -RC_CH_VALUE_OFFSET) {
                    motor_current_1 = motor_turn_speed(motor_device_list[CHASSIS_0],
                                                 (float) (RC_msg->rc.ch[Y]) / 660 * CHASSIS_SPEED);
                    motor_current_2 = motor_turn_speed(motor_device_list[CHASSIS_1],
                                                 (float) (-RC_msg->rc.ch[Y]) / 660 * CHASSIS_SPEED);
                    motor_current_3 = motor_turn_speed(motor_device_list[CHASSIS_2],
                                                 (float) (-RC_msg->rc.ch[Y]) / 660 * CHASSIS_SPEED);
                    motor_current_4 = motor_turn_speed(motor_device_list[CHASSIS_3],
                                                 (float) (RC_msg->rc.ch[Y]) / 660 * CHASSIS_SPEED);
                }
                else {
                    motor_current_1 = motor_turn_speed(motor_device_list[CHASSIS_0],
                                                 (float) (RC_msg->rc.ch[X]) / 660 * CHASSIS_SPEED);
                    motor_current_2 = motor_turn_speed(motor_device_list[CHASSIS_1],
                                                 (float) (RC_msg->rc.ch[X]) / 660 * CHASSIS_SPEED);
                    motor_current_3 = motor_turn_speed(motor_device_list[CHASSIS_2],
                                                 (float) (RC_msg->rc.ch[X]) / 660 * CHASSIS_SPEED);
                    motor_current_4 = motor_turn_speed(motor_device_list[CHASSIS_3],
                                                 (float) (RC_msg->rc.ch[X]) / 660 * CHASSIS_SPEED);
                }
            }
            else {
                motor_current_1 = motor_turn_speed(motor_device_list[CHASSIS_0], 0);;
                motor_current_2 = motor_turn_speed(motor_device_list[CHASSIS_1], 0);;
                motor_current_3 = motor_turn_speed(motor_device_list[CHASSIS_2], 0);;
                motor_current_4 = motor_turn_speed(motor_device_list[CHASSIS_3], 0);;
            }
        }



        /**************************************************************************************************************/
        /*电机使能*/

        fire_on(shoot_speed_pwm);
        CAN_cmd_gimbal(yaw_voltage, pitch_voltage + pitch_g_voltage, shoot_rate_current,0);
        CAN_cmd_chassis(motor_current_1, motor_current_2, motor_current_3, motor_current_4);

        /**************************************************************************************************************/
        /*串口输出(调试用)*/

        //Uart1->Print(Uart1, "%d,%d,%d,%d\n", (int)(100*INS_angle[0]), (int)(100*yaw_pose), (int)(100*yaw_speed), motor_device_list[YAW_M]->given_voltage(motor_device_list[YAW_M]));
        LED_RED_Toggle();


        vTaskDelay(1);
    }
}
