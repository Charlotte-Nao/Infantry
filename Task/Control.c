//
// Created by 14717 on 2025/8/27.
//

#include "Control.h"

#include "cmsis_os.h"

#include "../Devices/uart/dvc_uart.h"
#include "../Devices/remote/remote.h"

#define RC_CH_VALUE_MIN         ((uint16_t)0)
#define RC_CH_VALUE_OFFSET      ((uint16_t)220)
#define RC_CH_VALUE_MAX         ((uint16_t)660)

#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

struct uart_device *Uart1;
struct remote_device *RC;

const RC_ctrl_t *RC_msg;

typedef enum
{
    TURN = 0,
    MOVE = 1,
    YAW = 2,
    PITCH = 3,
    SHOOTSPEED = 4,
} channel;

typedef enum
{
    SHOOTRATE = 0,
} Switch;

int8_t turn_direction;
int16_t move_speed;
int8_t yaw_direction;
int8_t pitch_direction;
int16_t shoot_rate;
int8_t shoot_speed;

void Contorl_task(void const * argument) {

    RC = remote_get_device("remote");
    RC->init(RC);
    RC_msg = get_remote_control_point();
    Uart1 = uart_get_device("uart1_dma");
    Uart1->Init(Uart1, 115200, 8, 'N', 1);

    while (1) {

        Uart1->Print(Uart1, "%d\r\n", RC_msg->rc.ch[4]);

        if (RC_msg->rc.ch[YAW] > RC_CH_VALUE_OFFSET) {
            yaw_direction = 1;
        }
        else if (RC_msg->rc.ch[YAW] < -RC_CH_VALUE_OFFSET) {
            yaw_direction = -1;
        }
        else {
            yaw_direction = 0;
        }

        if (RC_msg->rc.ch[PITCH] > RC_CH_VALUE_OFFSET) {
            pitch_direction = 1;
        }
        else if (RC_msg->rc.ch[PITCH] < -RC_CH_VALUE_OFFSET) {
            pitch_direction = -1;
        }
        else {
            pitch_direction = 0;
        }

        shoot_speed = RC_msg->rc.ch[SHOOTSPEED];

        shoot_rate = (4 - RC_msg->rc.s[SHOOTRATE]) / 3;

        vTaskDelay(10);
    }

}