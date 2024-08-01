#include "bl616_glb.h"
#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "bflb_i2c.h"
#include "usbd_core.h"
#include "bflb_mtimer.h"
#include "board.h"
#include "ina229.h"
#include "tca9534.h"
#include "gw1n.h"

#include <stdarg.h>  // For va_list

#define GPIO_LED GPIO_PIN_3
#define BOOT_PIN GPIO_PIN_2

struct bflb_device_s *gpio;
struct bflb_device_s *i2c0;
struct bflb_device_s *spi0;

extern void cdc_acm_init(void);
extern void cdc_acm_printf(const char *format, ...);
extern void tca9534_test(void);

void gpio_init(void)
{
    gpio = bflb_device_get_by_name("gpio");

    /* status led */
    bflb_gpio_init(gpio, GPIO_LED, GPIO_OUTPUT | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_set(gpio, GPIO_LED);

    /* boot button */
    bflb_gpio_init(gpio, BOOT_PIN, GPIO_INPUT | GPIO_SMT_EN | GPIO_DRV_0);

    /* i2c0_scl */
    bflb_gpio_init(gpio, GPIO_PIN_16, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    /* i2c0_sda */
    bflb_gpio_init(gpio, GPIO_PIN_17, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
}

void i2c0_init(void)
{
    i2c0 = bflb_device_get_by_name("i2c0");
    bflb_i2c_init(i2c0, 400000);
}

int main(void)
{
    board_init();
    gpio_init();
    i2c0_init();
    tca9534_init();

    //ina229_init();

    /* initialize usb cdc acm */
    cdc_acm_init();

    cdc_acm_printf("Press button to start\r\n");

    /* Wait for user press boot button */
    while(!bflb_gpio_read(gpio, BOOT_PIN));
    while(bflb_gpio_read(gpio, BOOT_PIN));

    cdc_acm_printf("Start program...\r\n");
    bflb_gpio_reset(gpio, GPIO_LED);

    cdc_acm_printf("Power on FPGA\r\n");
    gowin_power_off();
    gowin_power_on();
    bflb_mtimer_delay_ms(200);
    gowin_fpga_config();

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
