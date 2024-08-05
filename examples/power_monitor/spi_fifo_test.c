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

extern void cdc_acm_printf(const char *format, ...);

static volatile uint8_t data_valid = 0;

static void gpio1_isr(uint8_t pin)
{
    if (pin == GPIO_PIN_1) {
        data_valid = 1;
    }
}

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

    /* GPIO20 as input (txfifo_empty_a) */
    bflb_gpio_init(gpio, GPIO_PIN_20, GPIO_INPUT | GPIO_SMT_EN | GPIO_DRV_0);

    /* configure MCU_IO1 as external interrupt gpio (rdata_valid_a) */
    bflb_irq_disable(gpio->irq_num);
    bflb_gpio_init(gpio, GPIO_PIN_1, GPIO_INPUT | GPIO_SMT_EN);
    bflb_gpio_int_init(gpio, GPIO_PIN_1, GPIO_INT_TRIG_MODE_SYNC_RISING_EDGE);
    bflb_gpio_irq_attach(GPIO_PIN_1, gpio1_isr);
    bflb_irq_enable(gpio->irq_num);
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
    uint8_t p_tx[5] = {0x00};
    p_tx[0] = byte;
    bflb_spi_poll_exchange(spi0, p_tx, NULL, 1);
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

void spi_ctrl_cmd_write_data_len(uint16_t len)
{
    uint8_t p_tx[4] = {0x02, 0x00, 0x00, 0x00};
    uint8_t p_rx[4] = {0x00, 0x00, 0x00, 0x00};

    p_tx[1] = (uint8_t)(len >> 8);
    p_tx[2] = (uint8_t)(len);

    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);
    cdc_acm_printf("Write dt len/ack  = %d %d\r\n", len, p_rx[3]);
}

void spi_ctrl_cmd_read_data_len(void)
{
    uint8_t p_tx[4] = {0x03, 0x00, 0x00, 0x00};
    uint8_t p_rx[4] = {0x00, 0x00, 0x00, 0x00};
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);
    cdc_acm_printf("Read data len     = %d\r\n", (uint16_t)((p_rx[2] << 8) | p_rx[3]));
}

void spi_ctrl_cmd_write_data(void)
{
    uint8_t p_tx[20] = {0x04, 0x00, 0x00, 0x00, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    //bflb_spi_poll_exchange(spi0, p_tx, NULL, 20);
    bflb_spi_poll_exchange(spi0, p_tx, NULL, 8);
}

void spi_ctrl_cmd_read_data(void)
{
    uint8_t p_tx[8] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t p_rx[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 8);
    cdc_acm_printf("Read data         = %X %X %X %X\r\n", p_rx[4], p_rx[5], p_rx[6], p_rx[7]);
}

void spi_ctrl_cmd_read_tx_fifo_level(void)
{
    uint8_t p_tx[4] = {0x08, 0x00, 0x00, 0x00};
    uint8_t p_rx[4] = {0x00, 0x00, 0x00, 0x00};
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);
    cdc_acm_printf("Tx fifo level     = %d\r\n", (uint16_t)((p_rx[2] << 16) | p_rx[3]));
}

void spi_ctrl_cmd_read_rx_fifo_level(void)
{
    uint8_t p_tx[4] = {0x09, 0x00, 0x00, 0x00};
    uint8_t p_rx[4] = {0x00, 0x00, 0x00, 0x00};
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);
    cdc_acm_printf("Rx fifo level     = %d\r\n", (uint16_t)((p_rx[2] << 16) | p_rx[3]));
}

//  Write data with a given data length:
//
//  | cmd (0x0A) | len_h | len_l | dummy     | data1 | data2 | datan ..|
//                               | ACK(0x01) |

void spi_ctrl_cmd_write_data_with_given_data_len(void)
{
    uint8_t p_tx[20] = {0x0A, 0x00, 0x10, 0x00, 0xAF, 0xAE, 0xAD, 0xAC, 0xAB, 0xAA, 0xA9, 0xA8, 0xA7, 0xA6, 0xA5, 0xA4, 0xA3, 0xA2, 0xA1, 0xA0};
    bflb_spi_poll_exchange(spi0, p_tx, NULL, 20);
}

void spi_ctrl_data_receive_loop(void)
{
    uint8_t p_tx[20];
    uint8_t p_rx[20];

    uint16_t len;

    cdc_acm_printf("Running data recieving loop...\r\n");

    while(1)
    {
        if(data_valid)
        {
            /* command read data len */
            p_tx[0] = 0x03;
            bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);
            len = (uint16_t)((p_rx[2] << 8) | p_rx[3]);
            cdc_acm_printf("Read data len = %d\r\n", len);

            /* read data */
            p_tx[0] = 0x05;
            bflb_spi_poll_exchange(spi0, p_tx, p_rx, len + 4);
            cdc_acm_printf("Read data     = ");
            for(int i = 0; i < len; i++)
            {
                cdc_acm_printf("%2X ", p_rx[4 + i]);
            }
            cdc_acm_printf("\r\n");

            spi_ctrl_cmd_read_rx_fifo_level();

            /* small delay */
            bflb_mtimer_delay_ms(1);

            /* wait for tx fifo empty */
            while(!bflb_gpio_read(gpio, GPIO_PIN_20));

            cdc_acm_printf("Resend data\r\n");
            
            /* write data to the oponent */
            spi_ctrl_cmd_write_data_with_given_data_len();

            data_valid = 0;
        }
    }
}
