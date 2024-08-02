#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "bflb_mtimer.h"
#include "board.h"
#include <stdio.h>
#include <string.h>

static struct bflb_device_s *spi0;
static struct bflb_device_s *gpio;

extern void cdc_acm_printf(const char *format, ...);

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

void sspi_test(uint8_t cmd)
{
    uint8_t p_tx[4];

    p_tx[0] = cmd;
    p_tx[1] = 0;
    p_tx[2] = 0;
    p_tx[3] = 0;
    bflb_spi_poll_exchange(spi0, p_tx, NULL, 4);
}

void spi_ctrl_send_byte(uint8_t byte)
{
    uint8_t p_tx[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    p_tx[0] = byte;

    bflb_spi_poll_exchange(spi0, p_tx, NULL, 4);
}

void spi_ctrl_cmd_read_gw_version(void)
{
    uint8_t p_tx[5] = {0x06, 0x00, 0x00, 0x00, 0x00};
    uint8_t p_rx[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 5);

    cdc_acm_printf("GW version YYMMDD = %d %d %d\r\n", p_rx[2], p_rx[3], p_rx[4]);
}

void spi_ctrl_cmd_read_chip_id(void)
{
    uint8_t p_tx[5] = {0x07, 0x00, 0x00, 0x00, 0x00};
    uint8_t p_rx[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 5);

    cdc_acm_printf("Chip ID           = %X %X %X\r\n", p_rx[2], p_rx[3], p_rx[4]);
}

void spi_ctrl_cmd_reset_fifo(void)
{
    uint8_t p_tx[3] = {0x01, 0x00, 0x00};
    uint8_t p_rx[3] = {0x00, 0x00, 0x00};

    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 3);

    cdc_acm_printf("Reset fifo ack    = %X\r\n", p_rx[2]);
}

void spi_ctrl_cmd_write_data_len(void)
{
    uint8_t p_tx[4] = {0x02, 0xAA, 0x55, 0x00};
    uint8_t p_rx[4] = {0x00, 0x00, 0x00, 0x00};

    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);

    cdc_acm_printf("Write dt len ack  = %X\r\n", p_rx[3]);
}

void spi_ctrl_cmd_read_data_len(void)
{
    uint8_t p_tx[4] = {0x03, 0x00, 0x00, 0x00};
    uint8_t p_rx[4] = {0x00, 0x00, 0x00, 0x00};

    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);

    cdc_acm_printf("Read data len     = %X %X\r\n", p_rx[2], p_rx[3]);
}

void spi_ctrl_cmd_write_data(void)
{
    uint8_t p_tx[8] = {0x04, 0x00, 0x00, 0x00, 0x11, 0x22, 0x33, 0x44};
    bflb_spi_poll_exchange(spi0, p_tx, NULL, 8);
}

void spi_ctrl_cmd_read_data(void)
{
    uint8_t p_tx[8] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t p_rx[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 8);
    cdc_acm_printf("Read data len     = %X %X\r\n", p_rx[2], p_rx[3]);
    cdc_acm_printf("Read data         = %X %X %X %X\r\n", p_rx[4], p_rx[5], p_rx[6], p_rx[7]);
}
