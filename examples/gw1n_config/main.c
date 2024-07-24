#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "usbd_core.h"
#include "bflb_mtimer.h"
#include "board.h"
#include "gw1n_image.h"

#define GPIO_LED GPIO_PIN_27

#define clr_cs_pin() bflb_gpio_reset(gpio, GPIO_PIN_12)
#define set_cs_pin() bflb_gpio_set(gpio, GPIO_PIN_12)

struct bflb_device_s *gpio;
struct bflb_device_s *spi0;

extern void cdc_acm_init(void);
extern void cdc_acm_data_send_with_dtr_test(void);

void gpio_led_init(void)
{
    gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, GPIO_LED, GPIO_OUTPUT | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_set(gpio, GPIO_LED);
}

void spi_gpio_init(void)
{
    gpio = bflb_device_get_by_name("gpio");

    /* spi cs as gpio */
    bflb_gpio_init(gpio, GPIO_PIN_12, GPIO_OUTPUT | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_set(gpio, GPIO_PIN_12);

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
    bflb_spi_feature_control(spi0, SPI_CMD_SET_CS_INTERVAL, 0);
    bflb_spi_feature_control(spi0, SPI_CMD_SET_DATA_WIDTH, SPI_DATA_WIDTH_8BIT);
}

void spi_dummy_clk(void)
{
    bflb_spi_poll_exchange(spi0, NULL, NULL, 1);
}

uint32_t gowin_read(uint32_t cmd)
{
	uint8_t txData[8];
	uint8_t rxData[8];
	uint32_t ret;

	/* Prepare command buffer */
	txData[0] = (uint8_t)(cmd >> 24);
	txData[1] = (uint8_t)(cmd >> 16);
	txData[2] = (uint8_t)(cmd >> 8);
	txData[3] = (uint8_t)cmd;

    spi_dummy_clk();

    clr_cs_pin();
    bflb_spi_poll_exchange(spi0, txData, rxData, 8);
    set_cs_pin();

	ret = ((uint32_t)(rxData[4] << 24) |
		   (uint32_t)(rxData[5] << 16) |
		   (uint32_t)(rxData[6] << 8)  |
		   (uint32_t)(rxData[7]));

	return ret;
}

void gowin_write_cmd1(uint8_t cmd)
{
	uint8_t txData[1];

	/* Prepare command buffer */
	txData[0] = (uint8_t)(cmd);

    spi_dummy_clk();
    clr_cs_pin();
    bflb_spi_poll_exchange(spi0, txData, NULL, 1);
    set_cs_pin();
}

void gowin_write_cmd2(uint16_t cmd)
{
	uint8_t txData[2];

	/* Prepare command buffer */
	txData[0] = (uint8_t)(cmd >> 8);
	txData[1] = (uint8_t)(cmd);

    spi_dummy_clk();
    clr_cs_pin();
    bflb_spi_poll_exchange(spi0, txData, NULL, 2);
    set_cs_pin();
}

void gowin_download_bitstream(uint8_t *data, uint32_t len)
{
	uint8_t cmd = 0x3B;

    /* Send write command */
    spi_dummy_clk();
    clr_cs_pin();
    bflb_spi_poll_exchange(spi0, &cmd, NULL, 1);
    bflb_spi_poll_exchange(spi0, data, NULL, len);
    set_cs_pin();
}

void read_ipc_core_hw_version(void)
{
    uint8_t cmd[4] = {0x06, 0x06, 0x06, 0x06};
    uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};

    clr_cs_pin();
    bflb_spi_poll_exchange(spi0, cmd, data, 4);
    set_cs_pin();

    printf("IPC Core HW version ymd: %2d %2d %2d\r\n", data[1], data[2], data[3]);
}

int main(void)
{
    uint32_t data;

    board_init();
    gpio_led_init();
    spi_gpio_init();
    spi_init(2);

    printf("Gowin FPGA programming\r\n");

    /* Wait for fpga power is stable */
    bflb_mtimer_delay_ms(200);
    data = gowin_read(0x11000000);

    if(data != 0x900281B) {
    	printf("Error! Invalid device ID %X\r\n", data);
    	printf("Exit program\r\n");
    	return;
    } else {
    	printf("Found device ID %X\r\n", data);
    }

    /* Write enable */
    gowin_write_cmd2(0x1500);

    /* Write bitstream */
    gowin_download_bitstream((uint8_t*)gw1n_image, sizeof(gw1n_image));

    /* Write disable */
    gowin_write_cmd2(0x3A00);

    /* Write nop */
    gowin_write_cmd1(0x02);
    bflb_mtimer_delay_ms(10);

#if 0 /* Because we configured SSPI as GPIO pins so at the end we cannot access the SSPI bus */
    /* Validate the downloaded bit stream */
    data = gowin_read(0x41000000);

    if (data != 0x1F020) {
        printf("Error! Bit stream download failed %X\r\n", data);
    } else {
        printf("Bit stream download successfully\r\n");
    }
#endif

    /* Wait sometimes */
    bflb_mtimer_delay_ms(200);
    /* Read IPC core hw version */
    read_ipc_core_hw_version();

    cdc_acm_init();

    while (1) {
        cdc_acm_data_send_with_dtr_test();
        bflb_mtimer_delay_ms(1000);
        bflb_gpio_set(gpio, GPIO_LED);
        bflb_mtimer_delay_ms(1000);
        bflb_gpio_reset(gpio, GPIO_LED);
    }
}
