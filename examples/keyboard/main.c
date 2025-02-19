#include "bflb_mtimer.h"
#include "bflb_gpio.h"
#include "board.h"

#define DBG_TAG "MAIN"
#include "log.h"

extern void hid_keyboard_init(void);
extern void hid_keyboard_test(void);

struct bflb_device_s *gpio;

int main(void)
{
    board_init();
    hid_keyboard_init();
    gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, GPIO_PIN_10, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_0);

    bflb_mtimer_delay_ms(1000);
    LOG_F("start hid keyboard\r\n");

    while(1) {
        while (!bflb_gpio_read(gpio, GPIO_PIN_10)) {
            hid_keyboard_test();
            bflb_mtimer_delay_ms(20);
            //LOG_F("Enter press\r\n");
        }
        //LOG_F("Enter press relesed\r\n");
        bflb_mtimer_delay_ms(200);
    }
}
