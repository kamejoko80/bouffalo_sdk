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

    /* spi cs as gpio */
    bflb_gpio_init(gpio, GPIO_PIN_28, GPIO_OUTPUT | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_set(gpio, GPIO_PIN_28);
    /* spi clk */
    bflb_gpio_init(gpio, GPIO_PIN_29, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_SMT_EN | GPIO_DRV_1);
    /* spi miso */
    bflb_gpio_init(gpio, GPIO_PIN_30, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_SMT_EN | GPIO_DRV_1);
    /* spi mosi */
    bflb_gpio_init(gpio, GPIO_PIN_27, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_SMT_EN | GPIO_DRV_1);
}

void i2c0_init(void)
{
    i2c0 = bflb_device_get_by_name("i2c0");
    bflb_i2c_init(i2c0, 400000);
}

void spi0_init(uint8_t baudmhz)
{
    struct bflb_spi_config_s spi_cfg = {
        .freq = baudmhz * 1000 * 1000,
        .role = SPI_ROLE_MASTER,
        .mode = SPI_MODE0,
        .data_width = SPI_DATA_WIDTH_8BIT,
        .bit_order = SPI_BIT_MSB,
        .byte_order = SPI_BYTE_LSB,
        .tx_fifo_threshold = 0,
        .rx_fifo_threshold = 0,
    };

    spi0 = bflb_device_get_by_name("spi0");
    bflb_spi_init(spi0, &spi_cfg);
    bflb_spi_feature_control(spi0, SPI_CMD_SET_CS_INTERVAL, 0);
    bflb_spi_feature_control(spi0, SPI_CMD_SET_DATA_WIDTH, SPI_DATA_WIDTH_8BIT);
}

int main(void)
{
    board_init();
    gpio_init();
    i2c0_init();
    tca9534_init();
    spi0_init(20);

    //ina229_init();

    /* initialize usb cdc acm */
    cdc_acm_init();

    cdc_acm_printf("Press button to start\r\n");

    /* Wait for user press boot button */
    while(!bflb_gpio_read(gpio, BOOT_PIN));

    cdc_acm_printf("Start program...\r\n");
    bflb_mtimer_delay_ms(2000);
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
