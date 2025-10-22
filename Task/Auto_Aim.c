//
// Created by 14717 on 2025/9/21.
//

#include "Auto_Aim.h"
#include "cmsis_os.h"

#include "../Driver/uart/dvc_uart.h"
#include "../Driver/usb_cdc/usb_cdc.h"

target_info_t target_info;
bool_t target_ok;
struct uart_device *Uart1;
struct usb_device *Usb;

void Auto_Aim_task(void const *argument) {
    Uart1 = uart_get_device("uart1_dma");
    Uart1->Init(Uart1, 115200, 8, 'N', 1);
    Usb = usb_get_device();
    Usb->Init(Usb);

    auto_aim_init(Usb);
    while (1) {
        if (parse_target_data(&target_info) == 1) {
            target_ok = 1;
        } else {
            target_ok = 0;
        }

        Usb->Print(Usb, "target_ok: %d\r\n", target_ok);

        vTaskDelay(10);
    }
}
