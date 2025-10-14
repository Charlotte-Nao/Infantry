//
// Created by 14717 on 2025/8/27.
//

#ifndef INFANTRY_01_CAN_CONTROL_MOTOR_H
#define INFANTRY_01_CAN_CONTROL_MOTOR_H

#include "../../Application/struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

/*电机ID设置 */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,
} can_msg_id_e;

extern void can_filter_init(void);

extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

extern void CAN_cmd_chassis_reset_ID(void);

extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern const struct motor_measure *get_yaw_gimbal_motor_measure_point(void);

extern const struct motor_measure *get_pitch_gimbal_motor_measure_point(void);

extern const struct motor_measure *get_trigger_motor_measure_point(void);

extern const struct motor_measure *get_chassis_motor_measure_point(uint8_t i);

#endif //INFANTRY_01_CAN_CONTROL_MOTOR_H