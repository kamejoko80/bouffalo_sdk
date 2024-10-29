/*
 * Copyright (C) 2024 Hery Dang
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "board.h"
#include "bflb_mtimer.h"
#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "debug.h"

static struct bflb_device_s *gpio;

#define clr_csn_pin() bflb_gpio_reset(gpio, GPIO_PIN_28)
#define set_csn_pin() bflb_gpio_set(gpio, GPIO_PIN_28)

#define clr_sck_pin() bflb_gpio_reset(gpio, GPIO_PIN_29)
#define set_sck_pin() bflb_gpio_set(gpio, GPIO_PIN_29)

#define clr_mosi_pin() bflb_gpio_reset(gpio, GPIO_PIN_27)
#define set_mosi_pin() bflb_gpio_set(gpio, GPIO_PIN_27)

#define read_miso_pin() bflb_gpio_read(gpio, GPIO_PIN_30)

void lmcxo2_spi0_gpio_bitbang_init(void)
{
    gpio = bflb_device_get_by_name("gpio");

    /* fpga_vddio_enan as gpio */
    bflb_gpio_init(gpio, GPIO_PIN_0, GPIO_OUTPUT | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_set(gpio, GPIO_PIN_0);

    /* fpga_vcore_ena as gpio */
    bflb_gpio_init(gpio, GPIO_PIN_1, GPIO_OUTPUT | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_reset(gpio, GPIO_PIN_1);

    /* spi cs as gpio */
    bflb_gpio_init(gpio, GPIO_PIN_28, GPIO_OUTPUT | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_set(gpio, GPIO_PIN_28);
    /* spi clk as gpio */
    bflb_gpio_init(gpio, GPIO_PIN_29, GPIO_OUTPUT | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_reset(gpio, GPIO_PIN_29);
    /* spi miso as gpio */
    bflb_gpio_init(gpio, GPIO_PIN_30, GPIO_INPUT | GPIO_SMT_EN | GPIO_DRV_1);
    /* spi mosi as gpio */
    bflb_gpio_init(gpio, GPIO_PIN_27, GPIO_OUTPUT | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_reset(gpio, GPIO_PIN_27);
}

// SPI mode 0: CPOL = 0, CPHA = 0
// SPI Bit-Bang Driver: Full-Duplex, MSB First, 8-bit data width
static void spi_gpio_transfer(uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        uint8_t tx_byte = tx_buf ? tx_buf[i] : 0x00;
        uint8_t rx_byte = 0;

        for (int bit = 7; bit >= 0; bit--)
        {
            if((tx_byte >> bit) & 0x01)
            {
                set_mosi_pin();
            } else {
                clr_mosi_pin();
            }

            set_sck_pin();

            if (rx_buf)
            {
                rx_byte |= (read_miso_pin() << bit);
            }

            clr_sck_pin();
        }

        if (rx_buf)
        {
            rx_buf[i] = rx_byte;
        }
    }
}

static void lmcxo2_power_on(void)
{
    /* fpga_vcore_ena = 1 */
    bflb_gpio_set(gpio, GPIO_PIN_1);
    /* fpga_vddio_enan = 0 */
    bflb_gpio_reset(gpio, GPIO_PIN_0);
}

static void lmcxo2_power_off(void)
{
    /* fpga_vddio_enan = 1 */
    bflb_gpio_set(gpio, GPIO_PIN_0);
    /* fpga_vcore_ena = 0 */
    bflb_gpio_reset(gpio, GPIO_PIN_1);
}

static void spi_dummy_clk(uint32_t n_clk)
{
    spi_gpio_transfer(NULL, NULL, n_clk);
}

static uint32_t lmcxo2_read(uint32_t cmd)
{
	uint8_t txData[8];
	uint8_t rxData[8];
	uint32_t ret;

	/* Prepare command buffer */
	txData[0] = (uint8_t)(cmd >> 24);
	txData[1] = (uint8_t)(cmd >> 16);
	txData[2] = (uint8_t)(cmd >> 8);
	txData[3] = (uint8_t)cmd;

    spi_dummy_clk(1);

    clr_csn_pin();
    spi_gpio_transfer(txData, rxData, 8);
    set_csn_pin();

	ret = ((uint32_t)(rxData[4] << 24) |
		   (uint32_t)(rxData[5] << 16) |
		   (uint32_t)(rxData[6] << 8)  |
		   (uint32_t)(rxData[7]));

	return ret;
}

static void lmcxo2_write_cmd1(uint8_t cmd)
{
	uint8_t txData[1];

	/* Prepare command buffer */
	txData[0] = (uint8_t)(cmd);

    spi_dummy_clk(1);
    clr_csn_pin();
    spi_gpio_transfer(txData, NULL, 1);
    set_csn_pin();
}

static void lmcxo2_write_cmd2(uint16_t cmd)
{
	uint8_t txData[2];

	/* Prepare command buffer */
	txData[0] = (uint8_t)(cmd >> 8);
	txData[1] = (uint8_t)(cmd);

    spi_dummy_clk(1);
    clr_csn_pin();
    spi_gpio_transfer(txData, NULL, 2);
    set_csn_pin();
}

static void lmcxo2_download_bitstream(uint8_t *data, uint32_t len)
{
	uint8_t cmd = 0x3B;

    /* Send write command */
    spi_dummy_clk(1);
    clr_csn_pin();
    spi_gpio_transfer(&cmd, NULL, 1);
    spi_gpio_transfer(data, NULL, len);
    set_csn_pin();
}

void lmcxo2_fpga_config(void)
{
    uint32_t data;

    usb_printf("Gowin FPGA programming\r\n");

    /* Init dedicated spi for the FPGA config */
    lmcxo2_spi0_gpio_bitbang_init();

    /* power off fpga */
    lmcxo2_power_off();

    /* power on fpga */
    lmcxo2_power_on();

    /* wait for fpga por */
    bflb_mtimer_delay_ms(200);

    data = lmcxo2_read(0x11000000);

    if(data != 0x900281B) {
        usb_printf("Error! Invalid device ID %X\r\n", data);
        return;
    } else {
        usb_printf("Found device ID %X\r\n", data);
    }

    /* Write enable */
    lmcxo2_write_cmd2(0x1500);

    /* Write bitstream */
    //lmcxo2_download_bitstream((uint8_t*)gw1n_image, sizeof(gw1n_image));

    /* Write disable */
    lmcxo2_write_cmd2(0x3A00);

    /* Write nop */
    lmcxo2_write_cmd1(0x02);
    bflb_mtimer_delay_ms(10);

    usb_printf("Gowin fpga configuration done\r\n");

#if 0 /* Because we configured SSPI as GPIO pins so at the end we cannot access the SSPI bus */
    /* Validate the downloaded bit stream */
    data = lmcxo2_read(0x41000000);

    if (data != 0x1F020) {
        cdc_acm_printf("Error! Bit stream download failed %X\r\n", data);
    } else {
        cdc_acm_printf("Bit stream download successfully\r\n");
    }
#endif
}
