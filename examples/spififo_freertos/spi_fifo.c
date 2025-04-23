#include <FreeRTOS.h>
#include "semphr.h"
#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "bflb_mtimer.h"
#include "board.h"
#include <stdio.h>
#include <string.h>

#define DBG_TAG "SPI_FIFO"
#include "log.h"

static TaskHandle_t spi_fifo_task_handle;

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
 *             (IO12) - FPGA_IO15  // txfifo_empty_a
 *
 * MODULE B connection:
 *
 *    SPI_CLK  (IO29) - FPGA_IO3
 *    SPI_MOSI (IO27) - FPGA_IO9
 *    SPI_MISO (IO30) - FPGA_IO10
 *    SPI_CSN  (IO28) - FPGA_IO12
 *
 *             (IO0)  - FPGA_IO4  // rdata_valid_b
 *             (IO1)  - FPGA_IO5  // txfifo_empty_b
 *             (IO3)  - FPGA_IO11
 *             (IO11) - FPGA_IO8
 *             (IO12) - FPGA_IO13
 *             (IO14) - FPGA_IO7
 *             (IO15) - FPGA_IO6
 *             (IO17) - FPGA_IO2
 *             (IO20) - FPGA_IO1
 */

#if defined(MCU_MODULE_A)
#define read_txfifo_empty() bflb_gpio_read(gpio, GPIO_PIN_12)
#else
#define read_txfifo_empty() bflb_gpio_read(gpio, GPIO_PIN_1)
#endif

static struct bflb_device_s *spi0;
static struct bflb_device_s *gpio;

static void rdata_valid_isr(uint8_t pin)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#if defined(MCU_MODULE_A)
    if (pin == GPIO_PIN_3) {
        vTaskNotifyGiveFromISR(spi_fifo_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);       
    }
#else
    if (pin == GPIO_PIN_0) {
        vTaskNotifyGiveFromISR(spi_fifo_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);         
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
    LOG_I("GW version YYMMDD = %d %d %d\r\n", p_rx[2], p_rx[3], p_rx[4]);
}

void spi_ctrl_cmd_read_chip_id(void)
{
    uint8_t p_tx[5] = {0x07, 0x00, 0x00, 0x00, 0x00};
    uint8_t p_rx[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 5);

    LOG_I("Chip ID           = %X %X %X\r\n", p_rx[2], p_rx[3], p_rx[4]);
}

void spi_ctrl_cmd_reset_fifo(void)
{
    uint8_t p_tx[3] = {0x01, 0x00, 0x00};
    uint8_t p_rx[3] = {0x00, 0x00, 0x00};
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 3);
    LOG_I("Reset fifo ack    = %X\r\n", p_rx[2]);
}

void spi_ctrl_cmd_write_data_len(uint16_t len)
{
    uint8_t p_tx[4] = {0x02, 0x00, 0x00, 0x00};
    uint8_t p_rx[4] = {0x00, 0x00, 0x00, 0x00};

    p_tx[1] = (uint8_t)(len >> 8);
    p_tx[2] = (uint8_t)(len);

    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);
    LOG_I("Write dt len/ack  = %d %d\r\n", len, p_rx[3]);
}

void spi_ctrl_cmd_read_data_len(void)
{
    uint8_t p_tx[4] = {0x03, 0x00, 0x00, 0x00};
    uint8_t p_rx[4] = {0x00, 0x00, 0x00, 0x00};
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);
    LOG_I("Read data len     = %d\r\n", (uint16_t)((p_rx[2] << 8) | p_rx[3]));
}

void spi_ctrl_cmd_write_data(void)
{
    uint8_t p_tx[20] = {0x04, 0x00, 0x00, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0xA5};
    bflb_spi_poll_exchange(spi0, p_tx, NULL, 20);
    LOG_I("Write data\r\n");
}

//  Write data with a given data length:
//
//  | cmd (0x0A) | len_h | len_l | dummy     | data1 | data2 | datan ..|
//                               | ACK(0x01) |

void spi_ctrl_cmd_write_data_with_given_data_len(void)
{
    uint8_t p_tx[20] = {0x0A, 0x00, 0x10, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0xA5};
    bflb_spi_poll_exchange(spi0, p_tx, NULL , 20);
}

void spi_ctrl_cmd_read_data(void)
{
    uint8_t p_tx[8] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t p_rx[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 8);
    LOG_I("Read data         = %X %X %X %X\r\n", p_rx[4], p_rx[5], p_rx[6], p_rx[7]);
}

void spi_ctrl_cmd_read_tx_fifo_level(void)
{
    uint8_t p_tx[4] = {0x08, 0x00, 0x00, 0x00};
    uint8_t p_rx[4] = {0x00, 0x00, 0x00, 0x00};
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);
    LOG_I("Tx fifo level     = %d\r\n", (uint16_t)((p_rx[2] << 16) | p_rx[3]));
}

void spi_ctrl_cmd_read_rx_fifo_level(void)
{
    uint8_t p_tx[4] = {0x09, 0x00, 0x00, 0x00};
    uint8_t p_rx[4] = {0x00, 0x00, 0x00, 0x00};
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);
    LOG_I("Rx fifo level     = %d\r\n", (uint16_t)((p_rx[2] << 16) | p_rx[3]));
}

static void spi_fifo_task(void *pvParameters)
{
    uint8_t p_tx[20];
    uint8_t p_rx[20];

    uint16_t len;
    
    LOG_I("spi fifo task enter\r\n");

    for (;;)
    {
        /* Clear notification on receipt */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        /* command read data len */
        p_tx[0] = 0x03;
        bflb_spi_poll_exchange(spi0, p_tx, p_rx, 4);
        len = (uint16_t)((p_rx[2] << 8) | p_rx[3]);
        printf("Read data len = %d\r\n", len);

        /* read data */
        p_tx[0] = 0x05;
        bflb_spi_poll_exchange(spi0, p_tx, p_rx, len + 4);
        printf("Read data     = ");
        for(int i = 0; i < len; i++)
        {
            printf("%2X ", p_rx[4 + i]);
        }
        printf("\r\n");

        spi_ctrl_cmd_read_rx_fifo_level();

        /* small delay */
        vTaskDelay(pdMS_TO_TICKS(1));

        /* wait for tx fifo empty */
        while(!read_txfifo_empty())
        {
            printf("tx fifo is not empty\r\n");
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        printf("Resend data\r\n");

        /* write data to the oponent */
        spi_ctrl_cmd_write_data_with_given_data_len();                
    }

    vTaskDelete(NULL);
}

void spi_fifo_task_init(void)
{
    LOG_I("[OS] Starting spi fifo task...\r\n");
    xTaskCreate(spi_fifo_task, (char *)"spi_fifo_task", 512, NULL, configMAX_PRIORITIES - 2, &spi_fifo_task_handle);    
}
