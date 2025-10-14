#include "fire_control.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

void fire_off(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1000);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1000);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1000);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 1000);
}

void fire_init(void) {
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_Base_Start(&htim8);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    fire_off();
}

void fire_on(uint16_t cmd)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, cmd);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, cmd);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, cmd);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, cmd);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, cmd);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, cmd);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, cmd);
}


