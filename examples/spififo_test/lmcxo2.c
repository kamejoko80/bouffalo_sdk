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

#define clr_progn_pin() bflb_gpio_reset(gpio, GPIO_PIN_17)
#define set_progn_pin() bflb_gpio_set(gpio, GPIO_PIN_17)
#define read_init_pin() bflb_gpio_read(gpio, GPIO_PIN_20)
#define read_done_pin() bflb_gpio_read(gpio, GPIO_PIN_11)


void lmcxo2_spi0_gpio_bitbang_init(void)
{
    gpio = bflb_device_get_by_name("gpio");

    /* fpga_vddio_enan as gpio */
    bflb_gpio_init(gpio, GPIO_PIN_0, GPIO_OUTPUT | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_set(gpio, GPIO_PIN_0);

    /* fpga_vcore_ena as gpio */
    bflb_gpio_init(gpio, GPIO_PIN_1, GPIO_OUTPUT | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_reset(gpio, GPIO_PIN_1);

    /* INITN gpio as input */
    bflb_gpio_init(gpio, GPIO_PIN_20, GPIO_INPUT | GPIO_SMT_EN | GPIO_DRV_0);
    /* DONE gpio as input */
    bflb_gpio_init(gpio, GPIO_PIN_11, GPIO_INPUT | GPIO_SMT_EN | GPIO_DRV_0);
    /* PROGRAMN gpio as output */
    bflb_gpio_init(gpio, GPIO_PIN_17, GPIO_OUTPUT | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_set(gpio, GPIO_PIN_17);

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
    /* fpga_vddio_enan = 0 */
    bflb_gpio_reset(gpio, GPIO_PIN_0);
    /* fpga_vcore_ena = 1 */
    bflb_gpio_set(gpio, GPIO_PIN_1);
}

static void lmcxo2_power_off(void)
{
    /* fpga_vcore_ena = 0 */
    bflb_gpio_reset(gpio, GPIO_PIN_1);
    /* fpga_vddio_enan = 1 */
    bflb_gpio_set(gpio, GPIO_PIN_0);
}

static void lmcxo2_read_device_id(void)
{
    uint32_t device_id;
    uint8_t cmd[] = {0xE0, 0x00, 0x00, 0x00};
    uint8_t id[]  = {0x00, 0x00, 0x00, 0x00};

    clr_csn_pin();
    spi_gpio_transfer(cmd, id, 4);
    set_csn_pin();

    device_id = (id[1] << 16) | (id[2] << 8) | id[3];
    usb_printf("Device ID: 0x%06X\r\n", device_id);
}

void lmcxo2_fpga_config(void)
{
    usb_printf("LMCXO2 FPGA programming\r\n");

    /* Init dedicated spi for the FPGA config */
    lmcxo2_spi0_gpio_bitbang_init();

    /* power off fpga */
    lmcxo2_power_off();
    /* power on fpga */
    lmcxo2_power_on();

    /* program pulse */
    bflb_mtimer_delay_ms(10);
    clr_progn_pin();
    bflb_mtimer_delay_ms(10);
    set_progn_pin();
    bflb_mtimer_delay_ms(10);

    /* read fpga device id */
    lmcxo2_read_device_id();

    return;
}
