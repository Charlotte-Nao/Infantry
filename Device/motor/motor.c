//
// Created by 14717 on 2025/8/27.
//

#include <string.h>

#include "motor.h"
#include "math.h"

#define pi (fp32)M_PI


/**********************************************************************************************************************/
/*私有数据定义*/
struct motor_data {
    /*计数值*/
    uint8_t round_num;
    /*反馈报文*/
    uint16_t ecd;
    int16_t current;
    int16_t speed_rpm;
    uint8_t temperate;
    /*状态值*/
    uint16_t last_ecd;
    /*直观数据*/
    fp32 relative_angle;
    fp32 relative_speed;
    /*目标值*/
    fp32 target_angle;
    fp32 target_speed;
    int16_t target_ecd;
    /*控制值*/
    int16_t given_current;
    int16_t given_voltage; //3508和2006不用
    /*PID*/
    struct PID *pid1;
    struct PID *pid2;
};
/**********************************************************************************************************************/
/*通用函数*/
static void motor_init(struct motor_device *motor, struct PID *pid1, struct PID *pid2) {
    struct motor_data *pData = motor->motor_data;
    //电机数据初始化
    pData->round_num = 0;
    pData->ecd = 0;
    pData->given_current = 0;
    pData->given_voltage = 0;
    pData->last_ecd = 0;
    pData->speed_rpm = 0;
    pData->temperate = 0;
    pData->relative_angle = 0;
    pData->relative_speed = 0;
    pData->target_angle = 0;
    pData->target_speed = 0;
    pData->pid1 = pid1;
    pData->pid2 = pid2;
}

static void motor_get_measure(struct motor_device *motor, uint8_t *data) {
    struct motor_data *pData = motor->motor_data;
    pData->last_ecd = pData->ecd;
    (pData)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
    pData->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);
    (pData)->given_current = (int16_t)((data)[4] << 8 | (data)[5]);
    (pData)->temperate = data[6];
}

static void motor_set_speed(struct motor_device *motor, fp32 speed) {
    struct motor_data *pData = motor->motor_data;
    pData->target_speed = speed;
}

static void motor_set_current(struct motor_device *motor, int16_t current) {
    struct motor_data *pData = motor->motor_data;
    pData->given_current = current;
}

static void motor_set_angle(struct motor_device *motor, fp32 angle) {
    struct motor_data *pData = motor->motor_data;
    pData->target_angle = rad_format(angle);
}

static void motor_set_ecd(struct motor_device *motor, int16_t ecd) {
    struct motor_data *pData = motor->motor_data;
    pData->target_ecd = ecd;
}

static int16_t motor_get_ecd(struct motor_device *motor) {
    struct motor_data *pData = motor->motor_data;
    return pData->ecd;
}

static int16_t motor_get_speed(struct motor_device *motor) {
    struct motor_data *pData = motor->motor_data;
    return pData->speed_rpm;
}
static int16_t motor_get_current(struct motor_device *motor) {
    struct motor_data *pData = motor->motor_data;
    return pData->current;
}
static fp32 motor_get_angle(struct motor_device *motor) {
    struct motor_data *pData = motor->motor_data;
    return pData->relative_angle;
}
static fp32 motor_get_relative_speed(struct motor_device *motor) {
    struct motor_data *pData = motor->motor_data;
    return pData->relative_speed;
}
static int16_t motor_get_temperature(struct motor_device *motor) {
    struct motor_data *pData = motor->motor_data;
    return pData->temperate;
}
static int16_t motor_given_current(struct motor_device *motor) {
    struct motor_data *pData = motor->motor_data;
    return pData->given_current;
}

/**********************************************************************************************************************/
/*3508*/
/*位置环更新*/
static void motor_3508_angle_update(struct motor_device *motor) {
    struct motor_data *pData = motor->motor_data;
    /*计算圈数*/
    if (pData->ecd - pData->last_ecd > 4096) {
        pData->round_num--;
    }
    else if (pData->ecd - pData->last_ecd < -4096) {
        pData->round_num++;
    }
    /*计算直观数据*/
    pData->relative_angle = rad_format((float)(pData->ecd + pData->round_num * 8192) / 8192 / 19.02f * 2 * pi);
    pData->relative_speed = rad_format((float)(pData->ecd - pData->last_ecd)/8192 / 19.02f * 2 * pi);
    /*计算电流*/
    //pData->target_speed = pData->pid1->calc(pData->pid1, pData->relative_angle, pData->target_angle, pData->relative_speed);
    pData->given_current = (int16_t)pData->pid2->calc(pData->pid2, pData->relative_speed, pData->target_speed, 0);
}
static int16_t motor_3508_given_voltage(struct motor_device *motor) {
    struct motor_data *pData = motor->motor_data;
    return 0;
}

/**********************************************************************************************************************/
/*2006*/
/*位置环更新*/
static void motor_2006_angle_update(struct motor_device *motor) {
    struct motor_data *pData = motor->motor_data;
    /*计算圈数*/
    if (pData->ecd - pData->last_ecd > 4096) {
        pData->round_num--;
    }
    else if (pData->ecd - pData->last_ecd < -4096) {
        pData->round_num++;
    }
    /*计算直观数据*/
    pData->relative_angle = rad_format((float)(pData->ecd + pData->round_num * 8192) / 8192 / 36.0f * 2 * pi);
    pData->relative_speed = rad_format((float)(pData->ecd - pData->last_ecd)/8192 / 36.0f * 2 * pi);
    /*计算电流*/
    //pData->target_speed = pData->pid1->calc(pData->pid1, pData->relative_angle, pData->target_angle, pData->relative_speed);
    pData->given_current = (int16_t)pData->pid2->calc(pData->pid2, pData->relative_speed, pData->target_speed, 0);
}
static int16_t motor_2006_given_voltage(struct motor_device *motor) {
    struct motor_data *pData = motor->motor_data;
    return 0;
}

/**********************************************************************************************************************/
/*6020*/
/*位置环更新*/
static void motor_6020_speed_update_voltage(struct motor_device *motor) {
    struct motor_data *pData = motor->motor_data;
    /*计算直观数据*/
    pData->relative_angle = rad_format((float)pData->ecd / 8192 * 2 * pi);
    pData->relative_speed = rad_format((float)(pData->ecd - pData->last_ecd)/8192 * 2 * pi);
    /*计算电流*/
    //pData->target_speed = pData->pid1->calc(pData->pid1, pData->relative_angle, pData->target_angle, pData->relative_speed);
    pData->given_voltage = (int16_t)pData->pid2->calc(pData->pid2, pData->relative_speed, pData->target_speed, 0);
}

static void motor_6020_angle_update_voltage(struct motor_device *motor) {
    struct motor_data *pData = motor->motor_data;
    /*计算直观数据*/
    pData->relative_angle = rad_format((float)pData->ecd / 8192 * 2 * pi);
    pData->relative_speed = rad_format((float)(pData->ecd - pData->last_ecd)/8192 * 2 * pi);
    /*计算电流*/
    pData->target_speed = pData->pid1->calc(pData->pid1, pData->relative_angle, pData->target_angle, pData->relative_speed);
    pData->given_voltage = (int16_t)pData->pid2->calc(pData->pid2, pData->relative_speed, pData->target_speed, 0);
}

static int16_t motor_6020_given_voltage(struct motor_device *motor) {
    struct motor_data *pData = motor->motor_data;
    return pData->given_voltage;
}
/**********************************************************************************************************************/
/*3508实例*/
struct motor_data chassis_motor1_data = {
    .ecd = 0,
    .given_current = 0,
    .given_voltage = 0,
    .last_ecd = 0,
    .speed_rpm = 0,
    .temperate = 0,
    .relative_angle = 0,
    .relative_speed = 0,
    .target_angle = 0,
    .target_speed = 0,
    .target_ecd = 0,
    .pid1 = NULL,
    .pid2 = NULL
};
struct motor_device chassis_motor1 = {
    "chassis_motor1",
    0,
    motor_init,
    motor_3508_angle_update,
    motor_get_measure,
    motor_set_speed,
    motor_set_current,
    motor_set_angle,
    motor_set_ecd,
    motor_get_ecd,
    motor_get_speed,
    motor_get_relative_speed,
    motor_get_current,
    motor_get_angle,
    motor_get_temperature,
    motor_given_current,
    motor_3508_given_voltage,
    &chassis_motor1_data
};
struct motor_data chassis_motor2_data = {
    .ecd = 0,
    .given_current = 0,
    .given_voltage = 0,
    .last_ecd = 0,
    .speed_rpm = 0,
    .temperate = 0,
    .relative_angle = 0,
    .relative_speed = 0,
    .target_angle = 0,
    .target_speed = 0,
    .target_ecd = 0,
    .pid1 = NULL,
    .pid2 = NULL
};
struct motor_device chassis_motor2 = {
    "chassis_motor2",
    1,
    motor_init,
    motor_3508_angle_update,
    motor_get_measure,
    motor_set_speed,
    motor_set_current,
    motor_set_angle,
    motor_set_ecd,
    motor_get_ecd,
    motor_get_speed,
    motor_get_relative_speed,
    motor_get_current,
    motor_get_angle,
    motor_get_temperature,
    motor_given_current,
    motor_3508_given_voltage,
    &chassis_motor2_data
};
struct motor_data chassis_motor3_data = {
    .ecd = 0,
    .given_current = 0,
    .given_voltage = 0,
    .last_ecd = 0,
    .speed_rpm = 0,
    .temperate = 0,
    .relative_angle = 0,
    .relative_speed = 0,
    .target_angle = 0,
    .target_speed = 0,
    .target_ecd = 0,
    .pid1 = NULL,
    .pid2 = NULL
};
struct motor_device chassis_motor3 = {
    "chassis_motor3",
    2,
    motor_init,
    motor_3508_angle_update,
    motor_get_measure,
    motor_set_speed,
    motor_set_current,
    motor_set_angle,
    motor_set_ecd,
    motor_get_ecd,
    motor_get_speed,
    motor_get_relative_speed,
    motor_get_current,
    motor_get_angle,
    motor_get_temperature,
    motor_given_current,
    motor_3508_given_voltage,
    &chassis_motor3_data
};
struct motor_data chassis_motor4_data = {
    .ecd = 0,
    .given_current = 0,
    .given_voltage = 0,
    .last_ecd = 0,
    .speed_rpm = 0,
    .temperate = 0,
    .relative_angle = 0,
    .relative_speed = 0,
    .target_angle = 0,
    .target_speed = 0,
    .target_ecd = 0,
    .pid1 = NULL,
    .pid2 = NULL
};
struct motor_device chassis_motor4 = {
    "chassis_motor4",
    3,
    motor_init,
    motor_3508_angle_update,
    motor_get_measure,
    motor_set_speed,
    motor_set_current,
    motor_set_angle,
    motor_set_ecd,
    motor_get_ecd,
    motor_get_speed,
    motor_get_relative_speed,
    motor_get_current,
    motor_get_angle,
    motor_get_temperature,
    motor_given_current,
    motor_3508_given_voltage,
    &chassis_motor4_data
};

/*6020实例*/
static struct motor_data motor_yaw_data = {
    .ecd = 0,
    .given_current = 0,
    .given_voltage = 0,
    .last_ecd = 0,
    .speed_rpm = 0,
    .temperate = 0,
    .relative_angle = 0,
    .relative_speed = 0,
    .target_angle = 0,
    .target_speed = 0,
    .target_ecd = 0,
    .pid1 = NULL,
    .pid2 = NULL
 };
static struct motor_device gimbal_motor_yaw = {
    "gimbal_motor_yaw",
    4,
    motor_init,
    motor_6020_speed_update_voltage,
    motor_get_measure,
    motor_set_speed,
    motor_set_current,
    motor_set_angle,
    motor_set_ecd,
    motor_get_ecd,
    motor_get_speed,
    motor_get_relative_speed,
    motor_get_current,
    motor_get_angle,
    motor_get_temperature,
    motor_given_current,
    motor_6020_given_voltage,
    &motor_yaw_data,
};
static struct motor_data motor_pitch_data = {
    .ecd = 0,
    .given_current = 0,
    .given_voltage = 0,
    .last_ecd = 0,
    .speed_rpm = 0,
    .temperate = 0,
    .relative_angle = 0,
    .relative_speed = 0,
    .target_angle = 0,
    .target_speed = 0,
    .target_ecd = 0,
    .pid1 = NULL,
    .pid2 = NULL
};
static struct motor_device gimbal_motor_pitch = {
    "gimbal_motor_pitch",
    5,
    motor_init,
    motor_6020_speed_update_voltage,
    motor_get_measure,
    motor_set_speed,
    motor_set_current,
    motor_set_angle,
    motor_set_ecd,
    motor_get_ecd,
    motor_get_speed,
    motor_get_relative_speed,
    motor_get_current,
    motor_get_angle,
    motor_get_temperature,
    motor_given_current,
    motor_6020_given_voltage,
    &motor_pitch_data,
};
/*2006实例*/
struct motor_data trigger_motor_data = {
    .ecd = 0,
    .given_current = 0,
    .given_voltage = 0,
    .last_ecd = 0,
    .speed_rpm = 0,
    .temperate = 0,
    .relative_angle = 0,
    .relative_speed = 0,
    .target_angle = 0,
    .target_speed = 0,
    .target_ecd = 0,
    .pid1 = NULL,
    .pid2 = NULL
};
struct motor_device trigger_motor = {
    "trigger_motor",
    6,
    motor_init,
    motor_2006_angle_update,
    motor_get_measure,
    motor_set_speed,
    motor_set_current,
    motor_set_angle,
    motor_set_ecd,
    motor_get_ecd,
    motor_get_speed,
    motor_get_relative_speed,
    motor_get_current,
    motor_get_angle,
    motor_get_temperature,
    motor_given_current,
    motor_2006_given_voltage,
    &trigger_motor_data
};
/**********************************************************************************************************************/
/*对外接口*/
struct motor_device *motor_list[] = {&chassis_motor1, &chassis_motor2, &chassis_motor3, &chassis_motor4, &gimbal_motor_yaw, &gimbal_motor_pitch, &trigger_motor};

struct motor_device *motor_get_device(const char *name)
{
    for (unsigned i = 0; i < sizeof(motor_list) / sizeof(motor_list[0]); ++i) {
        if (strcmp(motor_list[i]->name, name) == 0) {
            return motor_list[i];
        }
    }
    return NULL;
}


