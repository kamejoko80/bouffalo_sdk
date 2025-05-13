#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "bflb_dma.h"
#include "bflb_mtimer.h"
#include "board.h"
#include <stdio.h>
#include <string.h>
#include "debug.h"

/*
 * For the command list refer to the following code:
 * https://github.com/kamejoko80/litex-soc-builder/blob/main/custom_projects/test_spi_fifo_gw1n_fpga_evb.py
 */

/* MODULE A connection:
 *
 *    SPI_CLK  (IO29) - SSPI_CLK
 *    SPI_MOSI (IO27) - SSPI_MOSI
 *    SPI_MISO (IO30) - SSPI_MISO
 *    SPI_CSN  (IO28) - SSPI_CSN
 *
 *             (IO3)  - FPGA_IO14  // rdata_valid_a
 *             (IO12) - FPGA_IO15  // txfifo_full_a (has been removed in V2.0.0)
 *
 * MODULE B connection:
 *
 *    SPI_CLK  (IO29) - FPGA_IO3
 *    SPI_MOSI (IO27) - FPGA_IO9
 *    SPI_MISO (IO30) - FPGA_IO10
 *    SPI_CSN  (IO28) - FPGA_IO12
 *
 *             (IO0)  - FPGA_IO4  // rdata_valid_b
 *             (IO1)  - FPGA_IO5  // txfifo_full_b (has been removed in V2.0.0)
 *             (IO3)  - FPGA_IO11
 *             (IO11) - FPGA_IO8
 *             (IO12) - FPGA_IO13
 *             (IO14) - FPGA_IO7
 *             (IO15) - FPGA_IO6
 *             (IO17) - FPGA_IO2
 *             (IO20) - FPGA_IO1
 */

#define SPI_FIFO_SIZE (4096) /* This depends on FPGA GW implementation */

#if defined(MCU_MODULE_A)
#define read_txfifo_full() bflb_gpio_read(gpio, GPIO_PIN_12)
#define read_rdata_valid() bflb_gpio_read(gpio, GPIO_PIN_3)
#else
#define read_txfifo_full() bflb_gpio_read(gpio, GPIO_PIN_1)
#define read_rdata_valid() bflb_gpio_read(gpio, GPIO_PIN_0)
#endif

static struct bflb_device_s *spi0;
static struct bflb_device_s *gpio;

static volatile uint8_t data_valid = 0;

static ATTR_NOCACHE_NOINIT_RAM_SECTION uint8_t tx_buffer[SPI_FIFO_SIZE];
static ATTR_NOCACHE_NOINIT_RAM_SECTION uint8_t rx_buffer[SPI_FIFO_SIZE];

/*
 * Frame structure:
 *  _____ _____ ______ ______ ____________
 * |     |     |      |      |            |
 * | SOH | LH  | LL   | CRC8 |    DATA    |
 * |_____|_____|______|______|____________|
 *
 *  SOH  : uint8_t  : Start of header
 *  LEN  : uint16_t : Data length
 *  CRC8 : uint8_t  : Header CRC checksum
 *  DATA : uint8_t  : IPC frame data
 *
 */

#define FRAME_SOH (0xA5)

typedef struct frame_header_s
{
    uint8_t  soh;    /* Start of header     */
    uint16_t len;    /* Payload length      */
    uint8_t  crc8;   /* Header CRC checksum */
} __attribute__((packed, aligned(1))) frame_header_t;

uint8_t crc8(uint8_t *data, uint16_t len)
{
  unsigned crc = 0;
  int i, j;

  /* Using x^8 + x^2 + x + 1 polynomial */
  for (j = len; j; j--, data++)
  {
    crc ^= (*data << 8);

    for(i = 8; i; i--)
    {
      if (crc & 0x8000)
      {
        crc ^= (0x1070 << 3);
      }
      crc <<= 1;
    }
  }
  return (uint8_t)(crc >> 8);
}

void frame_header_create(frame_header_t *header, uint16_t len)
{
    /* creat frame header */
    header->soh  = FRAME_SOH;
    header->len  = len;
    header->crc8 = crc8((uint8_t *)header, 3);
}

bool frame_header_check(frame_header_t *header)
{
    bool ret = false;
    uint8_t crc = crc8((uint8_t *)header, 3);

    if(header->soh != FRAME_SOH) {
        printf("Error frame header SOH\r\n");
    }
    else if(crc != header->crc8) {
        printf("Error frame header: crc8 = 0x%X, header->crc8 = 0x%X\r\n", crc8, header->crc8);
    }
    else {
        ret = true;
    }

    return ret;
}

void frame_header_dump(frame_header_t *header)
{
    printf("===== Frame Header Info =====\r\n");
    printf("  soh  : 0x%X\r\n", header->soh);
    printf("  len  : %d\r\n",   header->len);
    printf("  crc8 : 0x%X\r\n", header->crc8);
}

static void rdata_valid_isr(uint8_t pin)
{

#if defined(MCU_MODULE_A)
    if (pin == GPIO_PIN_3) {
        data_valid = 1;
    }
#else
    if (pin == GPIO_PIN_0) {
        data_valid = 1;
    }
#endif
}

void spi_gpio_init(void)
{
    gpio = bflb_device_get_by_name("gpio");

    /* spi csn */
    bflb_gpio_init(gpio, GPIO_PIN_28, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_SMT_EN | GPIO_DRV_1);
    /* spi clk */
    bflb_gpio_init(gpio, GPIO_PIN_29, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_SMT_EN | GPIO_DRV_1);
    /* spi miso */
    bflb_gpio_init(gpio, GPIO_PIN_30, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_SMT_EN | GPIO_DRV_1);
    /* spi mosi */
    bflb_gpio_init(gpio, GPIO_PIN_27, GPIO_FUNC_SPI0 | GPIO_ALTERNATE | GPIO_SMT_EN | GPIO_DRV_1);

#if defined(MCU_MODULE_A)
    /* txfifo_empty_a as input */
    bflb_gpio_init(gpio, GPIO_PIN_12, GPIO_INPUT | GPIO_SMT_EN | GPIO_DRV_0);

    /* configure rdata_valid_a as external interrupt gpio */
    bflb_irq_disable(gpio->irq_num);
    bflb_gpio_init(gpio, GPIO_PIN_3, GPIO_INPUT | GPIO_SMT_EN);
    bflb_gpio_int_init(gpio, GPIO_PIN_3, GPIO_INT_TRIG_MODE_SYNC_RISING_EDGE);
    bflb_gpio_irq_attach(GPIO_PIN_3, rdata_valid_isr);
    bflb_irq_enable(gpio->irq_num);
#else
    /* txfifo_empty_b as input */
    bflb_gpio_init(gpio, GPIO_PIN_1, GPIO_INPUT | GPIO_SMT_EN | GPIO_DRV_0);

    /* configure rdata_valid_b as external interrupt gpio */
    bflb_irq_disable(gpio->irq_num);
    bflb_gpio_init(gpio, GPIO_PIN_0, GPIO_INPUT | GPIO_SMT_EN);
    bflb_gpio_int_init(gpio, GPIO_PIN_0, GPIO_INT_TRIG_MODE_SYNC_RISING_EDGE);
    bflb_gpio_irq_attach(GPIO_PIN_0, rdata_valid_isr);
    bflb_irq_enable(gpio->irq_num);
#endif
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

void spi_fifo_interface_bus_init(uint8_t baudmhz)
{
    spi_gpio_init();
    spi_init(baudmhz);
}

// Support command list:
//
//  1) Reset fifo:
//
//     | cmd (0x01) | dummy |
//                          | ACK(0x01) |
//
//  2) Write data:
//
//     | cmd (0x04) | dummy | dummy | dummy | data1 | data2 | datan ..|
//
//  3) Read data:
//
//     | cmd (0x05) | dummy | dummy | dummy |
//                                          | data1 | data2 | datan ..|
//
//  4) Read gateware version:
//
//     | cmd (0x06) | dummy |
//                          | yy | mm | dd |
//
//  5) Read chip ID:
//
//     | cmd (0x07) | dummy |
//                          | chip_id2 | chip_id1 | chip_id0 |
//
//  6) Read tx fifo level:
//
//     | cmd (0x08) | dummy |
//                          | level_h | level_l |
//
//  7) Read rx fifo level:
//
//     | cmd (0x09) | dummy |
//                          | level_h | level_l |
//

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

uint16_t spi_ctrl_cmd_read_tx_fifo_level(void)
{
    uint8_t p_tx[4] = {0x08, 0x00, 0x00, 0x00};
    uint8_t p_rx[4] = {0x00, 0x00, 0x00, 0x00};
    uint16_t ret;
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);
    ret = (uint16_t)((p_rx[2] << 16) | p_rx[3]);
    printf("Tx fifo level = %d\r\n", ret);
    return ret;
}

uint16_t spi_ctrl_cmd_read_rx_fifo_level(void)
{
    uint8_t p_tx[4] = {0x09, 0x00, 0x00, 0x00};
    uint8_t p_rx[4] = {0x00, 0x00, 0x00, 0x00};
    uint16_t ret;

    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);
    ret = (uint16_t)((p_rx[2] << 16) | p_rx[3]);
    printf("Rx fifo level = %d\r\n", ret);
    return ret;
}

uint16_t spi_ctrl_read_tx_fifo_free_space(void)
{
    uint8_t p_tx[4] = {0x08, 0x00, 0x00, 0x00};
    uint8_t p_rx[4] = {0x00, 0x00, 0x00, 0x00};
    uint16_t ret;
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);
    ret = (uint16_t)((p_rx[2] << 16) | p_rx[3]);

    if(ret < SPI_FIFO_SIZE) {
        ret = SPI_FIFO_SIZE - ret;
    } else {
        ret = 0;
    }

    printf("Tx fifo free space = %d\r\n", ret);
    return ret;
}

void spi_ctrl_cmd_read_data(void)
{
    uint8_t p_tx[8] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t p_rx[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 8);
    printf("Read data         = %X %X %X %X\r\n", p_rx[4], p_rx[5], p_rx[6], p_rx[7]);
}

void spi_fifo_write(uint8_t *data, uint16_t len)
{
    frame_header_t header;
    uint8_t wr_cmd[4] = {0x04, 0x00, 0x00, 0x00};
    uint16_t txfifo_free;

    if(len > (SPI_FIFO_SIZE - sizeof(frame_header_t) - 4)) {
        printf("Error data len is over dma buffer length\r\n");
        return;
    }

    /* create frame header */
    frame_header_create(&header, len);

    /* copy spi fifo command */
    memcpy((uint8_t *)&tx_buffer[0], wr_cmd, 4);

    /* copy header */
    memcpy((uint8_t *)&tx_buffer[4], (uint8_t *)& header, sizeof(frame_header_t));

    /* copy data */
    memcpy((uint8_t *)&tx_buffer[4 + sizeof(frame_header_t)], data, len);

    /* check spi tx fifo free space */
    txfifo_free = spi_ctrl_read_tx_fifo_free_space();

    if(txfifo_free < (sizeof(frame_header_t) + len)) {
        printf("Error not enough txfifo free space\r\n");
        return;
    }

    /* execute spi fifo write data command */
    bflb_spi_poll_exchange(spi0, tx_buffer, rx_buffer, 4 + sizeof(frame_header_t) + len);
}

void spi_fifo_read(void)
{
    frame_header_t *header;
    uint8_t p_tx[8] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t p_rx[8];

    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 8);

    header = (frame_header_t *)&p_rx[4];

    if (frame_header_check(header)) {
        printf("Read data len = %d\r\n", header->len);
        tx_buffer[0] = 0x05;
        bflb_spi_poll_exchange(spi0, tx_buffer, rx_buffer, header->len + 4);

        printf("Read data     = ");
        for(int i = 0; i < header->len; i++)
        {
            printf("%2X ", rx_buffer[4 + i]);
        }
        printf("\r\n");
    } else {
        printf("Reset fifo\r\n");
        spi_ctrl_cmd_reset_fifo();
    }
}

void ping_pong(void)
{
    while(read_rdata_valid())
    {
        spi_fifo_read();
        spi_ctrl_cmd_read_rx_fifo_level();
    }
}

uint8_t msg1[16] = {0xA5, 0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11};
uint8_t msg2[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0xA5};

void send_msg1(void)
{
   spi_fifo_write(msg1, 16);
}

void send_msg2(void)
{
   spi_fifo_write(msg2, 16);
}

void spi_ctrl_data_receive_loop(void)
{

    printf("Running data recieving loop...\r\n");

    while(1)
    {
        if(data_valid)
        {
            data_valid = 0;

            ping_pong();

            /* small delay */
            bflb_mtimer_delay_ms(10);

            printf("Resend data\r\n");

            /* write data to the oponent */
            #if defined(MCU_MODULE_A)
                send_msg1();
                send_msg2();
            #else
                send_msg1();
            #endif
        }
    }
}