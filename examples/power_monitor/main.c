#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "usbd_core.h"
#include "bflb_mtimer.h"
#include "board.h"
#include "ina229.h"

#define GPIO_LED GPIO_PIN_3

struct bflb_device_s *gpio;

extern void cdc_acm_init(void);
extern void cdc_acm_data_send_with_dtr_test(void);

void gpio_init(void)
{
    gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, GPIO_LED, GPIO_OUTPUT | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_set(gpio, GPIO_LED);
}

int main(void)
{
    board_init();
    gpio_init();

    ina229_init();

    cdc_acm_init();

    while (1) {
        cdc_acm_data_send_with_dtr_test();
        bflb_mtimer_delay_ms(1000);
        bflb_gpio_set(gpio, GPIO_LED);
        bflb_mtimer_delay_ms(1000);
        bflb_gpio_reset(gpio, GPIO_LED);
    }
}
