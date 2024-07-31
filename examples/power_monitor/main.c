#include "bl616_glb.h"
#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "usbd_core.h"
#include "bflb_mtimer.h"
#include "board.h"
#include "ina229.h"

#include <stdarg.h>  // For va_list

#define GPIO_LED GPIO_PIN_3
#define BOOT_PIN GPIO_PIN_2

struct bflb_device_s *gpio;

extern void cdc_acm_init(void);
extern void cdc_acm_printf(const char *format, ...);

void gpio_init(void)
{
    gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, GPIO_LED, GPIO_OUTPUT | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_set(gpio, GPIO_LED);

    bflb_gpio_init(gpio, BOOT_PIN, GPIO_INPUT | GPIO_SMT_EN | GPIO_DRV_0);
}

int main(void)
{
    board_init();
    gpio_init();

    //ina229_init();

    /* initialize usb cdc acm */
    cdc_acm_init();

    cdc_acm_printf("Press button to start\r\n");

    /* Wait for user press boot button */
    while(!bflb_gpio_read(gpio, BOOT_PIN));

    cdc_acm_printf("Start program...\r\n");
    bflb_mtimer_delay_ms(2000);
    bflb_gpio_reset(gpio, GPIO_LED);

    while (1) {
        /* Check if user press boot pin */
        if(bflb_gpio_read(gpio, BOOT_PIN))
        {
            /* wait for button release */
            while(bflb_gpio_read(gpio, BOOT_PIN));
            cdc_acm_printf("System will reset after 3s\r\n");

            for(int i = 0; i < 3; i++)
            {
                bflb_mtimer_delay_ms(500);
                bflb_gpio_reset(gpio, GPIO_LED);
                bflb_mtimer_delay_ms(500);
                bflb_gpio_set(gpio, GPIO_LED);
            }

            bflb_gpio_reset(gpio, GPIO_LED);
            cdc_acm_printf("System reset!\r\n");
            GLB_SW_System_Reset();
        }
    }
}
