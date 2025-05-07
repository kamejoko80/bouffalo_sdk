#include <FreeRTOS.h>
#include "semphr.h"
#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "bflb_dma.h"
#include "bflb_mtimer.h"
#include "board.h"
#include <stdio.h>
#include <string.h>

#define DBG_TAG                  "SPI_FIFO"
#define SPI_DMA_MAX_TRANSFER_LEN (1024)
#define DMA_TX_DONE_BIT          (1 << 0)
#define DMA_RX_DONE_BIT          (1 << 1)

#include "log.h"

static TaskHandle_t      xSpiFifoTaskHandle;
static SemaphoreHandle_t xSpiMutex;

/*
 * For the command list refer to the following code:
 * https://github.com/kamejoko80/litex-soc-builder/blob/main/custom_projects/test_spi_fifo_gw1n_fpga_evb.py
 * https://github.com/kamejoko80/litex-soc-builder/blob/main/custom_projects/test_spi_fifo_gw1n_dynamic_clk_fpga_evb.py
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
static struct bflb_device_s *dma0_ch0;
static struct bflb_device_s *dma0_ch1;

static struct bflb_dma_channel_lli_pool_s tx_llipool[1];
static struct bflb_dma_channel_lli_pool_s rx_llipool[1];
static struct bflb_dma_channel_lli_transfer_s tx_transfers[1];
static struct bflb_dma_channel_lli_transfer_s rx_transfers[1];

static ATTR_NOCACHE_NOINIT_RAM_SECTION uint8_t tx_buffer[256];
static ATTR_NOCACHE_NOINIT_RAM_SECTION uint8_t rx_buffer[256];

void dma0_ch0_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //LOG_I("spi dma tx done\r\n");
    vTaskNotifyGiveIndexedFromISR(xSpiFifoTaskHandle, 0, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void dma0_ch1_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //LOG_I("spi dma rx done\r\n");
    vTaskNotifyGiveIndexedFromISR(xSpiFifoTaskHandle, 1, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void rdata_valid_isr(uint8_t pin)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#if defined(MCU_MODULE_A)
    if (pin == GPIO_PIN_3) {
        vTaskNotifyGiveFromISR(xSpiFifoTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
#else
    if (pin == GPIO_PIN_0) {
        vTaskNotifyGiveFromISR(xSpiFifoTaskHandle, &xHigherPriorityTaskWoken);
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

    struct bflb_dma_channel_config_s tx_config = {
        .direction = DMA_MEMORY_TO_PERIPH,
        .src_req = DMA_REQUEST_NONE,
        .dst_req = DMA_REQUEST_SPI0_TX,
        .src_addr_inc = DMA_ADDR_INCREMENT_ENABLE,
        .dst_addr_inc = DMA_ADDR_INCREMENT_DISABLE,
        .src_burst_count = DMA_BURST_INCR1,
        .dst_burst_count = DMA_BURST_INCR1,
        .src_width = DMA_DATA_WIDTH_8BIT,
        .dst_width = DMA_DATA_WIDTH_8BIT,
    };

    struct bflb_dma_channel_config_s rx_config = {
        .direction = DMA_PERIPH_TO_MEMORY,
        .src_req = DMA_REQUEST_SPI0_RX,
        .dst_req = DMA_REQUEST_NONE,
        .src_addr_inc = DMA_ADDR_INCREMENT_DISABLE,
        .dst_addr_inc = DMA_ADDR_INCREMENT_ENABLE,
        .src_burst_count = DMA_BURST_INCR1,
        .dst_burst_count = DMA_BURST_INCR1,
        .src_width = DMA_DATA_WIDTH_8BIT,
        .dst_width = DMA_DATA_WIDTH_8BIT,
    };

    xSpiMutex = xSemaphoreCreateMutex();
    if (xSpiMutex == NULL) {
        LOG_E("spi mutex not created\r\n");
    }

    spi0 = bflb_device_get_by_name("spi0");
    bflb_spi_init(spi0, &spi_cfg);
    bflb_spi_feature_control(spi0, SPI_CMD_SET_CS_INTERVAL, 1);
    bflb_spi_feature_control(spi0, SPI_CMD_SET_DATA_WIDTH, SPI_DATA_WIDTH_8BIT);
    bflb_spi_link_txdma(spi0, true);
    bflb_spi_link_rxdma(spi0, true);

    dma0_ch0 = bflb_device_get_by_name("dma0_ch0");
    dma0_ch1 = bflb_device_get_by_name("dma0_ch1");

    bflb_dma_channel_init(dma0_ch0, &tx_config);
    bflb_dma_channel_init(dma0_ch1, &rx_config);

    bflb_dma_channel_irq_attach(dma0_ch0, dma0_ch0_isr, NULL);
    bflb_dma_channel_irq_attach(dma0_ch1, dma0_ch1_isr, NULL);

    tx_transfers[0].src_addr = (uint32_t)tx_buffer;
    tx_transfers[0].dst_addr = (uint32_t)DMA_ADDR_SPI0_TDR;
    tx_transfers[0].nbytes = 0;

    rx_transfers[0].src_addr = (uint32_t)DMA_ADDR_SPI0_RDR;
    rx_transfers[0].dst_addr = (uint32_t)rx_buffer;
    rx_transfers[0].nbytes = 0;
}

void spi_dma_transfer_setup(uint8_t *tx_data, uint32_t len)
{
    if (xSemaphoreTake(xSpiMutex, portMAX_DELAY) == pdTRUE) {
        memcpy(tx_buffer, tx_data, len);
        tx_transfers[0].nbytes = len;
        rx_transfers[0].nbytes = len;
        bflb_dma_channel_lli_reload(dma0_ch0, tx_llipool, 1, tx_transfers, 1);
        bflb_dma_channel_lli_reload(dma0_ch1, rx_llipool, 1, rx_transfers, 1);
        xSemaphoreGive(xSpiMutex);
    }
}

void spi_dma_transfer_start(void)
{
    if (xSemaphoreTake(xSpiMutex, portMAX_DELAY) == pdTRUE) {
        bflb_dma_channel_start(dma0_ch0);
        bflb_dma_channel_start(dma0_ch1);
        xSemaphoreGive(xSpiMutex);
    }
}

void spi_dma_transfer_wait_for_complete(void)
{
    xSpiFifoTaskHandle = xTaskGetCurrentTaskHandle();
    uint32_t notifyValue = 0;

    while ((notifyValue & (DMA_TX_DONE_BIT | DMA_RX_DONE_BIT)) != (DMA_TX_DONE_BIT | DMA_RX_DONE_BIT)) {
        ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY); // TX
        notifyValue |= DMA_TX_DONE_BIT;

        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY); // RX
        notifyValue |= DMA_RX_DONE_BIT;
    }
}

void spi_dma_transfer_execute(uint8_t *tx_data, uint32_t len)
{
    spi_dma_transfer_setup(tx_data, len);
    spi_dma_transfer_start();
    spi_dma_transfer_wait_for_complete();
}

void spi_fifo_interface_bus_init(uint8_t baudmhz)
{
    spi_gpio_init();
    spi_init(baudmhz);
}

void spi_ctrl_cmd_read_gw_version(void)
{
    uint8_t p_tx[5] = {0x06, 0x00, 0x00, 0x00, 0x00};
    spi_dma_transfer_execute(p_tx, 5);
    LOG_I("GW version YYMMDD = %d %d %d\r\n", rx_buffer[2], rx_buffer[3], rx_buffer[4]);
}

void spi_ctrl_cmd_read_chip_id(void)
{
    uint8_t p_tx[5] = {0x07, 0x00, 0x00, 0x00, 0x00};
    spi_dma_transfer_execute(p_tx, 5);
    LOG_I("Chip ID           = %X %X %X\r\n", rx_buffer[2], rx_buffer[3], rx_buffer[4]);
}

void spi_ctrl_cmd_reset_fifo(void)
{
    uint8_t p_tx[3] = {0x01, 0x00, 0x00};
    spi_dma_transfer_execute(p_tx, 3);
    LOG_I("Reset fifo ack    = %X\r\n", rx_buffer[2]);
}

void spi_ctrl_cmd_write_data_len(uint16_t len)
{
    uint8_t p_tx[4] = {0x02, 0x00, 0x00, 0x00};
    p_tx[1] = (uint8_t)(len >> 8);
    p_tx[2] = (uint8_t)(len);
    spi_dma_transfer_execute(p_tx, 4);
    LOG_I("Write dt len/ack  = %d %d\r\n", len, rx_buffer[3]);
}

void spi_ctrl_cmd_read_data_len(void)
{
    uint8_t p_tx[4] = {0x03, 0x00, 0x00, 0x00};
    spi_dma_transfer_execute(p_tx, 4);
    LOG_I("Read data len     = %d\r\n", (uint16_t)((rx_buffer[2] << 8) | rx_buffer[3]));
}

void spi_ctrl_cmd_write_data(void)
{
    uint8_t p_tx[20] = {0x04, 0x00, 0x00, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0xA5};
    spi_dma_transfer_execute(p_tx, 20);
    LOG_I("Write data\r\n");
}

//  Write data with a given data length:
//
//  | cmd (0x0A) | len_h | len_l | dummy     | data1 | data2 | datan ..|
//                               | ACK(0x01) |

void spi_ctrl_cmd_write_data_with_given_data_len(void)
{
    uint8_t p_tx[20] = {0x0A, 0x00, 0x10, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0xA5};
    spi_dma_transfer_execute(p_tx, 20);
}

void spi_ctrl_cmd_read_data(void)
{
    uint8_t p_tx[8] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    spi_dma_transfer_execute(p_tx, 8);
    LOG_I("Read data         = %X %X %X %X\r\n", rx_buffer[4], rx_buffer[5], rx_buffer[6], rx_buffer[7]);
}

void spi_ctrl_cmd_read_tx_fifo_level(void)
{
    uint8_t p_tx[4] = {0x08, 0x00, 0x00, 0x00};
    spi_dma_transfer_execute(p_tx, 4);
    LOG_I("Tx fifo level     = %d\r\n", (uint16_t)((rx_buffer[2] << 16) | rx_buffer[3]));
}

void spi_ctrl_cmd_read_rx_fifo_level(void)
{
    uint8_t p_tx[4] = {0x09, 0x00, 0x00, 0x00};
    spi_dma_transfer_execute(p_tx, 4);
    LOG_I("Rx fifo level     = %d\r\n", (uint16_t)((rx_buffer[2] << 16) | rx_buffer[3]));
}

static void spi_fifo_task(void *pvParameters)
{
    uint8_t p_tx[20];
    uint16_t len;

    LOG_I("spi fifo task enter\r\n");

    spi_ctrl_cmd_read_gw_version();
    spi_ctrl_cmd_read_chip_id();

#if !defined(MCU_MODULE_A)
    spi_ctrl_cmd_write_data_with_given_data_len();
#endif

    for (;;)
    {
        /* Clear notification on receipt */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* command read data len */
        memset((void *)p_tx, 0x00, 4);
        p_tx[0] = 0x03;

        spi_dma_transfer_execute(p_tx, 4);
        len = (uint16_t)((rx_buffer[2] << 8) | rx_buffer[3]);
        printf("Read data len = %d\r\n", len);

        /* read data */
        memset((void *)p_tx, 0x00, 20);
        p_tx[0] = 0x05;
        spi_dma_transfer_execute(p_tx, len + 4);

        printf("Read data     = ");
        for(int i = 0; i < len; i++)
        {
            printf("%2X ", rx_buffer[4 + i]);
        }
        printf("\r\n");

        spi_ctrl_cmd_read_rx_fifo_level();

        /* small delay */
        vTaskDelay(pdMS_TO_TICKS(1000));

        /* wait for tx fifo empty */
        while(!read_txfifo_empty())
        {
            printf("tx fifo is not empty\r\n");
            //vTaskDelay(pdMS_TO_TICKS(10));
            vTaskDelete(NULL); /* Terminate task */
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
    xTaskCreate(spi_fifo_task, (char *)"spi_fifo_task", 1024, NULL, configMAX_PRIORITIES - 2, &xSpiFifoTaskHandle);
}
