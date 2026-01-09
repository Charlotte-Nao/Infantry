#include "../ALL_Task/logic_task.h"
#include <stdio.h>
#include "../Bsp/uart/bsp_uart.h"
#include "../Application/robot_global.h"
#include "cmsis_os.h"

void logic_task_func(void const * argument) {
    // 1. 初始化调试串口 (确保波特率匹配)
    struct uart_device *Uart = uart_get_device("uart1_dma");
    if (Uart != NULL) {
        Uart->Init(Uart, 115200, 8, 'N', 1);
    }

    while (1) {
        if (robot_ctrl.rc == NULL) {
            if (Uart) Uart->Print(Uart, "[Error] Waiting for RC Data...\r\n");
            osDelay(500);
            continue;
        }

        if (Uart) {
            // --- 1. 遥控器通道与物理按键验证 ---
            // 包含：4通道摇杆、挡位(SW)、拨轮(Wh)、暂停(Pse)、自定义左/右(CuL/CuR)、扳机(Trig)
            Uart->Print(Uart, "RC >> CH0:%4d CH1:%4d CH2:%4d CH3:%4d | SW:%d | Wh:%4d | Pse:%d CuL:%d CuR:%d Trig:%d\r\n",
                        robot_ctrl.rc->rc.ch[0],
                        robot_ctrl.rc->rc.ch[1],
                        robot_ctrl.rc->rc.ch[2],
                        robot_ctrl.rc->rc.ch[3],
                        robot_ctrl.rc->rc.sw,
                        robot_ctrl.rc->rc.wheel,
                        robot_ctrl.rc->rc.pause,
                        robot_ctrl.rc->rc.custom_l,
                        robot_ctrl.rc->rc.custom_r,
                        robot_ctrl.rc->rc.trigger);

            // --- 2. 鼠标数据验证 ---
            // 包含：X/Y/Z轴速度、左/中/右键
            Uart->Print(Uart, "MS >> X:%5d Y:%5d Z:%5d | L:%d R:%d M:%d\r\n",
                        robot_ctrl.rc->mouse.x,
                        robot_ctrl.rc->mouse.y,
                        robot_ctrl.rc->mouse.z,
                        robot_ctrl.rc->mouse.press_l,
                        robot_ctrl.rc->mouse.press_r,
                        robot_ctrl.rc->mouse.press_m);

            // --- 3. 键盘数据验证 ---
            // 打印 16 进制原始值，并手动解析几个核心键位辅助观察
            uint16_t v = robot_ctrl.rc->key.v;
            Uart->Print(Uart, "KB >> Raw:0x%04X | W:%d S:%d A:%d D:%d Shift:%d Ctrl:%d\r\n",
                        v,
                        (v & KEY_W) ? 1 : 0,
                        (v & KEY_S) ? 1 : 0,
                        (v & KEY_A) ? 1 : 0,
                        (v & KEY_D) ? 1 : 0,
                        (v & KEY_SHIFT) ? 1 : 0,
                        (v & KEY_CTRL) ? 1 : 0);

            Uart->Print(Uart, "--------------------------------------------------------------------------------\r\n");
        }

        // 采样率建议设为 10Hz - 20Hz，太快了串口助手刷新不过来
        osDelay(100);
    }
}