//
// Created by 14717 on 2025/9/21.
//

#ifndef INFANTRY_01_AUTO_AIM_H
#define INFANTRY_01_AUTO_AIM_H

#include "../Application/auto_aim.h"
#include "../Application/struct_typedef.h"

extern void Auto_Aim_task(void const * argument);

extern target_info_t target_info;
extern bool_t target_ok;

#endif //INFANTRY_01_AUTO_AIM_H