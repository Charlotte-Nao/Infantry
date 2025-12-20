#include "FreeRTOS.h"
#include "task.h"
#include "../Bsp/can/bsp_can.h"
#include "../Components/BMI088/BMI088driver.h"
#include "../Components/ist8310_i2c/ist8310_driver.h"
#include "../Bsp/uart/bsp_uart.h"  // 确保包含你之前的串口驱动头文件
#include "../Bsp/LED/bsp_led.h"
#include "../Bsp/usb_cdc/bsp_usb_cdc.h"
#include "../Bsp/uart/bsp_uart.h"
#include "../Components/motor/motor.h"
#include "../Application/robot_global.h"

void test_task(void const * argument)
{
    //bsp_can_init();

    // vTaskDelay(1000);
    //
    // Motor_System_PowerOn_Init();
    //
    // struct motor_device *motor = motor_get_device("J4310_PITCH");
    //
    // float motor_status1;
    // float motor_status2;


    while (1) {



        // motor->get_status(motor, "POS", &motor_status1);
        // motor->get_status(motor, "p_des", &motor_status2);
        //
        // //motor->set_target(motor, 1, 500.0f);
        // //motor->set_target(motor, 1, -0.0f);
        //
        //
        // Motor_All_Update();
        // DJI_Motor_Send_CAN1_Group(&hcan1);
        // DJI_Motor_Send_CAN2_Group(&hcan2);
        // struct motor_device* PITCH = motor_get_device("J4310_PITCH");
        // PITCH->send_ctrl_cmd(PITCH);
        //
        //
        // //Uart->Print(Uart, "%f,%f\r\n", motor_status1, motor_status2);
        //Uart->Print(Uart, "%f,%f,%f\r\n", robot_ctrl.imu.yaw, robot_ctrl.imu.pitch, robot_ctrl.imu.roll);
        //Uart->Print(Uart, "%d,%d\r\n", robot_ctrl.rc->rc.ch[0], robot_ctrl.rc->rc.ch[1]);

        vTaskDelay(10000);
    }
}