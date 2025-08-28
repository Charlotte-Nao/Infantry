//
// Created by ReallyTired on 25-7-11.
//
#include<math.h>
#include "pid.h"

#define pi (fp32)M_PI

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



