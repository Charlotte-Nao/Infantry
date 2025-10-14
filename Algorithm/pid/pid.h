//
// Created by 14717 on 25-7-11.
//

#ifndef PID_H
#define PID_H

#endif //PID_H

#include "../../application/struct_typedef.h"

struct PID_data {
    //PID控制参数
    fp32 kp;
    fp32 ki;
    fp32 kd;

    //误差
    fp32 error;
    fp32 error_angle;

    //限幅
    fp32 max_iout;
    fp32 max_out;

    //控制量与当前量
    fp32 set;
    fp32 fdb;


    //pid输出
    fp32 out;
    fp32 pout;
    fp32 iout;
    fp32 dout;
};

struct PID {
    char *name;
    void (*init)(struct PID *pid, const fp32 PID[3], fp32 max_out, fp32 max_iout);
    fp32 (*calc)(struct PID *pid, fp32 ref, fp32 set, fp32 speed);
    struct PID_data *pid_data;
};

extern fp32 rad_format(fp32 rad);

extern struct PID *pid_get_device(const char *name);

