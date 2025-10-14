//
// Created by 14717 on 2025/9/21.
//

#include "Auto_Aim.h"
#include "cmsis_os.h"

#include "../Driver/uart/dvc_uart.h"

target_info_t target_info;
bool_t target_ok;
struct uart_device *Uart1;

void Auto_Aim_task(void const *argument) {
    Uart1 = uart_get_device("uart1_dma");
    Uart1->Init(Uart1, 115200, 8, 'N', 1);
    auto_aim_init(Uart1);
    while (1) {
        if (parse_target_data(&target_info) == 1) {
            target_ok = 1;
        } else {
            target_ok = 0;
        }
        vTaskDelay(10);
    }
}
