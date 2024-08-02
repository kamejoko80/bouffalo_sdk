#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "bflb_mtimer.h"
#include "board.h"
#include <stdio.h>
#include <string.h>

static struct bflb_device_s *spi0;
static struct bflb_device_s *gpio;

static void spi_gpio_init(void)
{
    gpio = bflb_device_get_by_name("gpio");

    /* spi cs */
    bflb_gpio_init(gpio, GPIO_PIN_28, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_SMT_EN | GPIO_DRV_1);
    /* spi clk */
    bflb_gpio_init(gpio, GPIO_PIN_29, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_SMT_EN | GPIO_DRV_1);
    /* spi miso */
    bflb_gpio_init(gpio, GPIO_PIN_30, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_SMT_EN | GPIO_DRV_1);
    /* spi mosi */
    bflb_gpio_init(gpio, GPIO_PIN_27, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_SMT_EN | GPIO_DRV_1);
}

static void spi_init(void)
{
    /* switch to auto csn asserting mode */
    spi0 = bflb_device_get_by_name("spi0");
    bflb_spi_feature_control(spi0, SPI_CMD_SET_CS_INTERVAL, 1);
}

void spi_fifo_interface_bus_init(void)
{
    spi_gpio_init();
    spi_init();
}

void spi_fifo_test(uint8_t cmd)
{
    uint8_t p_tx[4];

    p_tx[0] = cmd;
    p_tx[1] = 0;
    p_tx[2] = 0;
    p_tx[3] = 0;
    bflb_spi_poll_exchange(spi0, p_tx, NULL, 4);
}
