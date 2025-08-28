//
// Created by 14717 on 2025/8/27.
//

#ifndef INFANTRY_01_CONTROL_H
#define INFANTRY_01_CONTROL_H

#include "../Application/struct_typedef.h"

extern void Contorl_task(void const * argument);

extern int8_t turn_direction;
extern int16_t move_speed;
extern int8_t yaw_direction;
extern int8_t pitch_direction;
extern int16_t shoot_rate;
extern int8_t shoot_speed;


#endif //INFANTRY_01_CONTROL_H