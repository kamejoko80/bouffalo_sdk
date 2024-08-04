#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "bflb_mtimer.h"
#include "board.h"
#include <stdio.h>
#include <string.h>

/*
 * For the command list refer to the following code: 
 * https://github.com/kamejoko80/litex-soc-builder/blob/main/custom_projects/test_spi_fifo_gw1n_lv1.py
 */

static struct bflb_device_s *spi0;
static struct bflb_device_s *gpio;

void spi_gpio_init(void)
{
    gpio = bflb_device_get_by_name("gpio");

    /* spi csn */
    bflb_gpio_init(gpio, GPIO_PIN_12, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_SMT_EN | GPIO_DRV_1);
    /* spi clk */
    bflb_gpio_init(gpio, GPIO_PIN_13, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_SMT_EN | GPIO_DRV_1);
    /* spi miso */
    bflb_gpio_init(gpio, GPIO_PIN_14, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_SMT_EN | GPIO_DRV_1);
    /* spi mosi */
    bflb_gpio_init(gpio, GPIO_PIN_15, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_SMT_EN | GPIO_DRV_1);
}

void spi_init(uint8_t baudmhz)
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
    bflb_spi_feature_control(spi0, SPI_CMD_SET_CS_INTERVAL, 1);
    bflb_spi_feature_control(spi0, SPI_CMD_SET_DATA_WIDTH, SPI_DATA_WIDTH_8BIT);
}

void spi_fifo_interface_bus_init(void)
{
    spi_gpio_init();
    spi_init(30);
}

void spi_ctrl_send_byte(uint8_t byte)
{
    uint8_t p_tx[1] = {0x00};
    p_tx[0] = byte;
    bflb_spi_poll_exchange(spi0, p_tx, NULL, 1);
}

void spi_ctrl_cmd_read_gw_version(void)
{
    uint8_t p_tx[5] = {0x06, 0x00, 0x00, 0x00, 0x00};
    uint8_t p_rx[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 5);
    printf("GW version YYMMDD = %d %d %d\r\n", p_rx[2], p_rx[3], p_rx[4]);
}

void spi_ctrl_cmd_read_chip_id(void)
{
    uint8_t p_tx[5] = {0x07, 0x00, 0x00, 0x00, 0x00};
    uint8_t p_rx[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 5);

    printf("Chip ID           = %X %X %X\r\n", p_rx[2], p_rx[3], p_rx[4]);
}

void spi_ctrl_cmd_reset_fifo(void)
{
    uint8_t p_tx[3] = {0x01, 0x00, 0x00};
    uint8_t p_rx[3] = {0x00, 0x00, 0x00};
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 3);
    printf("Reset fifo ack    = %X\r\n", p_rx[2]);
}

void spi_ctrl_cmd_write_data_len(uint16_t len)
{
    uint8_t p_tx[4] = {0x02, 0x00, 0x00, 0x00};
    uint8_t p_rx[4] = {0x00, 0x00, 0x00, 0x00};
    
    p_tx[1] = (uint8_t)(len >> 8);
    p_tx[2] = (uint8_t)(len);
    
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);
    printf("Write dt len/ack  = %d %d\r\n", len, p_rx[3]);
}

void spi_ctrl_cmd_read_data_len(void)
{
    uint8_t p_tx[4] = {0x03, 0x00, 0x00, 0x00};
    uint8_t p_rx[4] = {0x00, 0x00, 0x00, 0x00};
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);
    printf("Read data len     = %d\r\n", (uint16_t)((p_rx[2] << 8) | p_rx[3]));
}

void spi_ctrl_cmd_write_data(void)
{
    uint8_t p_tx[20] = {0x04, 0x00, 0x00, 0x00, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26};
    bflb_spi_poll_exchange(spi0, p_tx, NULL, 20);   
    printf("Write data\r\n");
}

void spi_ctrl_cmd_read_data(void)
{
    uint8_t p_tx[8] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t p_rx[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 8);
    printf("Read data         = %X %X %X %X\r\n", p_rx[4], p_rx[5], p_rx[6], p_rx[7]);
}

void spi_ctrl_cmd_read_tx_fifo_level(void)
{
    uint8_t p_tx[4] = {0x08, 0x00, 0x00, 0x00};
    uint8_t p_rx[4] = {0x00, 0x00, 0x00, 0x00};
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);
    printf("Tx fifo level     = %d\r\n", (uint16_t)((p_rx[2] << 16) | p_rx[3]));    
}

void spi_ctrl_cmd_read_rx_fifo_level(void)
{
    uint8_t p_tx[4] = {0x09, 0x00, 0x00, 0x00};
    uint8_t p_rx[4] = {0x00, 0x00, 0x00, 0x00};
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);
    printf("Rx fifo level     = %d\r\n", (uint16_t)((p_rx[2] << 16) | p_rx[3]));    
}
