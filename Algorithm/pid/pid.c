//
// Created by ReallyTired on 25-7-11.
//
#include<math.h>
#include "pid.h"

#include "string.h"

#define pi (fp32)M_PI

/**********************************************************************************************************************/
/*数据处理*/

fp32 rad_format(fp32 rad) {
    if (rad > pi) {
        while (rad > pi) {
            rad = rad - 2 * pi;
        }
    }
    else if (rad < -pi) {
        while (rad < -pi) {
            rad = rad + 2 * pi;
        }
    }
    return rad;
}

/**********************************************************************************************************************/
/*电机PID*/

void Motor_PID_init(struct PID *pid, const fp32 PID[3], fp32 max_out, fp32 max_iout) {
    if (pid == NULL || PID == NULL) {
        return;
    }
    pid->pid_data->kp = PID[0];
    pid->pid_data->ki = PID[1];
    pid->pid_data->kd = PID[2];
    pid->pid_data->max_out = max_out;
    pid->pid_data->max_iout = max_iout;
    pid->pid_data->error = 0.0f;
    pid->pid_data->error_angle = 0.0f;
}

fp32 Motor_PID_calc(struct PID *pid, fp32 ref, fp32 set, fp32 speed){
    pid->pid_data->set = set;
    pid->pid_data->fdb = ref;
    pid->pid_data->error = rad_format(set - ref);
    pid->pid_data->error_angle = pid->pid_data->error * 180 / pi;

    //比例
    pid->pid_data->pout = pid->pid_data->kp * pid->pid_data->error;
    //积分
    pid->pid_data->iout += pid->pid_data->ki * pid->pid_data->error;
    pid->pid_data->iout = pid->pid_data->iout > pid->pid_data->max_iout ? pid->pid_data->max_iout : pid->pid_data->iout;
    pid->pid_data->iout = pid->pid_data->iout < -pid->pid_data->max_iout ? -pid->pid_data->max_iout : pid->pid_data->iout;
    //微分
    pid->pid_data->dout = pid->pid_data->kd * speed;

    pid->pid_data->out = pid->pid_data->pout + pid->pid_data->iout + pid->pid_data->dout;
    pid->pid_data->out = pid->pid_data->out > pid->pid_data->max_out ? pid->pid_data->max_out : pid->pid_data->out;
    pid->pid_data->out = pid->pid_data->out < -pid->pid_data->max_out ? -pid->pid_data->max_out : pid->pid_data->out;

    return pid->pid_data->out;
}

/**********************************************************************************************************************/
/*PID实例*/

struct PID_data motor_yaw_PID1_data = {
    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f
};

struct PID motor_yaw_PID1 = {
    "motor_yaw_PID1",
    Motor_PID_init,
    Motor_PID_calc,
    &motor_yaw_PID1_data,
};

struct PID_data motor_pitch_PID1_data = {
    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f
};

struct PID motor_pitch_PID1 = {
    "motor_pitch_PID1",
    Motor_PID_init,
    Motor_PID_calc,
    &motor_pitch_PID1_data,
};


struct PID_data motor_3508_0_PID2_data = {
    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f
};

struct PID motor_3508_0_PID2 = {
    "motor_3508_0_PID2",
    Motor_PID_init,
    Motor_PID_calc,
    &motor_3508_0_PID2_data,
};

struct PID_data motor_3508_1_PID2_data = {
    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f
};
struct PID motor_3508_1_PID2 = {
    "motor_3508_1_PID2",
    Motor_PID_init,
    Motor_PID_calc,
    &motor_3508_1_PID2_data,
};

struct PID_data motor_3508_2_PID2_data = {
    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f, 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f
};
struct PID motor_3508_2_PID2 = {
    "motor_3508_2_PID2",
    Motor_PID_init,
    Motor_PID_calc,
    &motor_3508_2_PID2_data,
};

struct PID_data motor_3508_3_PID2_data = {
    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f
};
struct PID motor_3508_3_PID2 = {
    "motor_3508_3_PID2",
    Motor_PID_init,
    Motor_PID_calc,
    &motor_3508_3_PID2_data,
};

struct PID_data motor_yaw_PID2_data = {
    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f
};

struct PID motor_yaw_PID2 = {
    "motor_yaw_PID2",
    Motor_PID_init,
    Motor_PID_calc,
    &motor_yaw_PID2_data,
};

struct PID_data motor_pitch_PID2_data = {
    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f
};

struct PID motor_pitch_PID2 = {
    "motor_pitch_PID2",
    Motor_PID_init,
    Motor_PID_calc,
    &motor_pitch_PID2_data,
};

struct PID_data motor_2006_PID2_data = {
    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f
};

struct PID motor_2006_PID2 = {
    "motor_2006_PID2",
    Motor_PID_init,
    Motor_PID_calc,
    &motor_2006_PID2_data,
};

struct PID_data yaw_keep_data = {
    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f
};

struct PID yaw_keep = {
    "yaw_keep",
    Motor_PID_init,
    Motor_PID_calc,
    &yaw_keep_data,
};

struct PID_data yaw_auto_data = {
    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f
};

struct PID yaw_auto = {
    "yaw_auto",
    Motor_PID_init,
    Motor_PID_calc,
    &yaw_auto_data,
};

struct PID_data pitch_keep_data = {
    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f
};

struct PID pitch_keep = {
    "pitch_keep",
    Motor_PID_init,
    Motor_PID_calc,
    &pitch_keep_data,
};

struct PID_data pitch_auto_data = {
    0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f
};

struct PID pitch_auto = {
    "pitch_auto",
    Motor_PID_init,
    Motor_PID_calc,
    &pitch_auto_data,
};

/**********************************************************************************************************************/
/*对外接口*/

/*对外接口*/
struct PID *PID_list[] = {
    &motor_yaw_PID1, &motor_pitch_PID1, &motor_yaw_PID2, &motor_pitch_PID2, &motor_2006_PID2,
    &motor_3508_0_PID2, &motor_3508_1_PID2, &motor_3508_2_PID2, &motor_3508_3_PID2,
    &yaw_keep, &yaw_auto, &pitch_keep, &pitch_auto
};

struct PID *pid_get_device(const char *name)
{
    for (unsigned i = 0; i < sizeof(PID_list) / sizeof(PID_list[0]); ++i) {
        if (strcmp(PID_list[i]->name, name) == 0) {
            return PID_list[i];
        }
    }
    return NULL;
}


