#include "systick.h"
#include "driver_usart1.h"

int main(void)
{
    systick_init(16000);
    usart1_init(115200);

    while (1) {
        usart1_send_string("test\r\n");
        delay_ms(500);
    }
}
