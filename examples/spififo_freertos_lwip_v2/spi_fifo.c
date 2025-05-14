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
#define SPI_DMA_MAX_TRANSFER_LEN (2048)
#define SPI_FIFO_SIZE            (4096) /* This depends on FPGA GW implementation */
#define DMA_TX_DONE_BIT          (1 << 0)
#define DMA_RX_DONE_BIT          (1 << 1)

#include "log.h"

static TaskHandle_t      xSpiFifoTaskHandle = NULL;
static TaskHandle_t      xSpiDmaTransferTaskHandle = NULL;
static SemaphoreHandle_t xSpiMutex;

/*
 * For the command list refer to the following code:
 * https://github.com/kamejoko80/litex-soc-builder/blob/main/custom_projects/test_spi_fifo_gw1n_fpga_evb.py
 * https://github.com/kamejoko80/litex-soc-builder/blob/main/custom_projects/test_spi_fifo_gw1n_dynamic_clk_fpga_evb.py
 * 
 * Note: This demo can only run with SPI FIFO V2.0.0 
 *
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

#if defined(MCU_MODULE_A)
#define read_rdata_valid()  bflb_gpio_read(gpio, GPIO_PIN_3)
#define read_txfifo_empty() bflb_gpio_read(gpio, GPIO_PIN_12)
#else
#define read_rdata_valid()  bflb_gpio_read(gpio, GPIO_PIN_0)    
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

static ATTR_NOCACHE_NOINIT_RAM_SECTION uint8_t tx_buffer[SPI_FIFO_SIZE];
static ATTR_NOCACHE_NOINIT_RAM_SECTION uint8_t rx_buffer[SPI_FIFO_SIZE];

/* gobal data tx/rx buffer */
uint8_t *p_tx = tx_buffer;
uint8_t *p_rx = rx_buffer;

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

void dma0_ch0_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //LOG_I("spi dma tx done\r\n");
    if(xSpiDmaTransferTaskHandle != NULL) {
        vTaskNotifyGiveIndexedFromISR(xSpiDmaTransferTaskHandle, 0, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void dma0_ch1_isr(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //LOG_I("spi dma rx done\r\n");
    if(xSpiDmaTransferTaskHandle != NULL) {
        vTaskNotifyGiveIndexedFromISR(xSpiDmaTransferTaskHandle, 1, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void rdata_valid_isr(uint8_t pin)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    LOG_I("rdata_valid_isr\r\n");
    if(xSpiFifoTaskHandle != NULL) {
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

void spi_dma_transfer_setup(uint8_t *tx_data, uint16_t len)
{
    if (xSemaphoreTake(xSpiMutex, portMAX_DELAY) == pdTRUE) {
        memcpy(tx_buffer, tx_data, len);
        if(len < SPI_FIFO_SIZE) {
            tx_transfers[0].nbytes = len;
            rx_transfers[0].nbytes = len;
        } else {
            tx_transfers[0].nbytes = SPI_FIFO_SIZE;
            rx_transfers[0].nbytes = SPI_FIFO_SIZE;
        }
        bflb_dma_channel_lli_reload(dma0_ch0, tx_llipool, 1, tx_transfers, 1);
        bflb_dma_channel_lli_reload(dma0_ch1, rx_llipool, 1, rx_transfers, 1);
        xSemaphoreGive(xSpiMutex);
    }
}

void spi_dma_transfer_read_setup(uint16_t len)
{
    if (xSemaphoreTake(xSpiMutex, portMAX_DELAY) == pdTRUE) {
        if(len < SPI_FIFO_SIZE) {
            tx_transfers[0].nbytes = len;
            rx_transfers[0].nbytes = len;
        } else {
            LOG_I("warning data len %d exceeds DMA buffer len %d\r\n", len, SPI_FIFO_SIZE);
            tx_transfers[0].nbytes = SPI_FIFO_SIZE;
            rx_transfers[0].nbytes = SPI_FIFO_SIZE;
        }
        bflb_dma_channel_lli_reload(dma0_ch0, tx_llipool, 1, tx_transfers, 1);
        bflb_dma_channel_lli_reload(dma0_ch1, rx_llipool, 1, rx_transfers, 1);
        xSemaphoreGive(xSpiMutex);
    }
}

void spi_dma_transfer_start(void)
{
    if (xSemaphoreTake(xSpiMutex, portMAX_DELAY) == pdTRUE) {
        /* must get task handle before start spi dma */
        xSpiDmaTransferTaskHandle = xTaskGetCurrentTaskHandle();
        bflb_dma_channel_start(dma0_ch0);
        bflb_dma_channel_start(dma0_ch1);
        xSemaphoreGive(xSpiMutex);
    }
}

void spi_dma_transfer_wait_for_complete(void)
{
    uint32_t notifyValue = 0;

    while ((notifyValue & (DMA_TX_DONE_BIT | DMA_RX_DONE_BIT)) != (DMA_TX_DONE_BIT | DMA_RX_DONE_BIT)) {
        ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY); // TX
        notifyValue |= DMA_TX_DONE_BIT;

        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY); // RX
        notifyValue |= DMA_RX_DONE_BIT;
    }
}

void spi_dma_transfer_execute(uint8_t *tx_data, uint16_t len)
{
    spi_dma_transfer_setup(tx_data, len);
    spi_dma_transfer_start();
    spi_dma_transfer_wait_for_complete();
}

void spi_dma_transfer_read_execute(uint16_t len)
{
    spi_dma_transfer_read_setup(len);
    spi_dma_transfer_start();
    spi_dma_transfer_wait_for_complete();
}

void spi_fifo_interface_bus_init(uint8_t baudmhz)
{
    spi_gpio_init();
    spi_init(baudmhz);
}

uint8_t rxdata_valid(void)
{
#if defined(MCU_MODULE_A)
    return bflb_gpio_read(gpio, GPIO_PIN_3);
#else
    return bflb_gpio_read(gpio, GPIO_PIN_0);
#endif    
}

// SPI FIFO V2.0.0 supports command list:
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
    uint8_t cmd[5] = {0x06, 0x00, 0x00, 0x00, 0x00};
    spi_dma_transfer_execute(cmd, 5);
    printf("GW version YYMMDD = %d %d %d\r\n", p_rx[2], p_rx[3], p_rx[4]);
}

void spi_ctrl_cmd_read_chip_id(void)
{
    uint8_t cmd[5] = {0x07, 0x00, 0x00, 0x00, 0x00};
    spi_dma_transfer_execute(cmd, 5);
    printf("Chip ID           = %X %X %X\r\n", p_rx[2], p_rx[3], p_rx[4]);
}

void spi_ctrl_cmd_reset_fifo(void)
{
    uint8_t cmd[3] = {0x01, 0x00, 0x00};
    spi_dma_transfer_execute(cmd, 3);
    //printf("Reset fifo ack    = %X\r\n", p_rx[2]);
}

uint16_t spi_ctrl_cmd_read_tx_fifo_level(void)
{
    uint8_t cmd[4] = {0x08, 0x00, 0x00, 0x00};
    uint16_t ret;
    spi_dma_transfer_execute(cmd, 4);
    ret = (uint16_t)((p_rx[2] << 8) | p_rx[3]);
    printf("Tx fifo level = %d\r\n", ret);
    return ret;
}

uint16_t spi_ctrl_cmd_read_rx_fifo_level(void)
{
    uint8_t cmd[4] = {0x09, 0x00, 0x00, 0x00};
    uint16_t ret;
    spi_dma_transfer_execute(cmd, 4);
    ret = (uint16_t)((p_rx[2] << 8) | p_rx[3]);
    printf("Rx fifo level = %d\r\n", ret);
    return ret;
}

uint16_t spi_ctrl_read_tx_fifo_free_space(void)
{
    uint8_t cmd[4] = {0x08, 0x00, 0x00, 0x00};
    uint16_t ret;
    spi_dma_transfer_execute(cmd, 4);
    ret = (uint16_t)((p_rx[2] << 8) | p_rx[3]);

    //printf("Tx fifo level = %d\r\n", ret);

    if(ret < SPI_FIFO_SIZE) {
        ret = SPI_FIFO_SIZE - ret;
    } else {
        ret = 0;
    }

    //printf("Tx fifo free space = %d\r\n", ret);
    return ret;
}

bool spi_fifo_write_with_check(uint8_t *data, uint16_t len)
{
    frame_header_t header;
    uint8_t wr_cmd[4] = {0x04, 0x00, 0x00, 0x00};
    uint16_t txfifo_free;

    if(len > (SPI_FIFO_SIZE - sizeof(frame_header_t) - 4)) {
        printf("Error data len is over dma buffer length\r\n");
        return false;
    }

    /* check spi tx fifo free space */
    txfifo_free = spi_ctrl_read_tx_fifo_free_space();

    if(txfifo_free < (sizeof(frame_header_t) + len)) {
        printf("Error not enough txfifo free space\r\n");
        return false;
    }

    if (xSemaphoreTake(xSpiMutex, portMAX_DELAY) == pdTRUE) {
        /* create frame header */
        frame_header_create(&header, len);
        /* copy spi fifo command */
        memcpy((uint8_t *)&tx_buffer[0], wr_cmd, 4);
        /* copy header */
        memcpy((uint8_t *)&tx_buffer[4], (uint8_t *)& header, sizeof(frame_header_t));
        /* copy data */
        memcpy((uint8_t *)&tx_buffer[4 + sizeof(frame_header_t)], data, len);
        xSemaphoreGive(xSpiMutex);
    }
    
    /* execute spi fifo write data command */
    spi_dma_transfer_read_execute(4 + sizeof(frame_header_t) + len);
 
    return true;
}

void spi_fifo_write(uint8_t *data, uint16_t len)
{
    frame_header_t header;
    uint8_t wr_cmd[4] = {0x04, 0x00, 0x00, 0x00};

    if (xSemaphoreTake(xSpiMutex, portMAX_DELAY) == pdTRUE) {
        /* create frame header */
        frame_header_create(&header, len);
        /* copy spi fifo command */
        memcpy((uint8_t *)&tx_buffer[0], wr_cmd, 4);
        /* copy header */
        memcpy((uint8_t *)&tx_buffer[4], (uint8_t *)& header, sizeof(frame_header_t));
        /* copy data */
        memcpy((uint8_t *)&tx_buffer[4 + sizeof(frame_header_t)], data, len);
        xSemaphoreGive(xSpiMutex);
    }
    
    /* execute spi fifo write data command */
    spi_dma_transfer_read_execute(4 + sizeof(frame_header_t) + len);
}

uint16_t spi_fifo_find_max_data_len(void)
{
    uint16_t txfifo_free;
    
    /* check spi tx fifo free space */
    txfifo_free = spi_ctrl_read_tx_fifo_free_space();

    if( txfifo_free >= sizeof(frame_header_t)) {
        return (txfifo_free - sizeof(frame_header_t));
    } else {
        return 0;
    }        
}

uint16_t spi_fifo_read(void)
{
    frame_header_t *header = NULL;
    uint8_t cmd[8] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint16_t len = 0;

    /* execure spi fifo read */
    spi_dma_transfer_execute(cmd, 8);

    if (xSemaphoreTake(xSpiMutex, portMAX_DELAY) == pdTRUE) {
        header = (frame_header_t *)&p_rx[4];
        len = header->len;
        xSemaphoreGive(xSpiMutex);        
    }
    
    /* read data payload */
    if (frame_header_check(header)) {
        // printf("Read data len = %d\r\n", len);
        tx_buffer[0] = 0x05;
        spi_dma_transfer_read_execute(len + 4);
        #if 0
            printf("Read data     = ");
            for(int i = 0; i < len; i++)
            {
                printf("%2X ", p_rx[4 + i]);
            }
            printf("\r\n");
        #endif
    } else {
        printf("Reset fifo\r\n");
        spi_ctrl_cmd_reset_fifo();
        len = 0;
    }
    return len;
}

extern void ethernetif_input(void *pvParameters);

void ethernetif_input_task_init(void *pvParameters)
{
    LOG_I("[OS] Starting ethernetif_input task...\r\n");
    xTaskCreate(ethernetif_input, (char *)"ethernetif_input", 2048, pvParameters, configMAX_PRIORITIES - 2, &xSpiFifoTaskHandle);
}
