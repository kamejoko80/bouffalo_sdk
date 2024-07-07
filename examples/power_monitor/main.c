#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "usbd_core.h"
#include "bflb_mtimer.h"
#include "board.h"

#define BUFF_LEN (8 * 1024)
#define GPIO_LED GPIO_PIN_3

uint32_t tx_buff[BUFF_LEN / 4];
uint32_t rx_buff[BUFF_LEN / 4];

struct bflb_device_s *spi0;
struct bflb_device_s *gpio;

extern void cdc_acm_init(void);
extern void cdc_acm_data_send_with_dtr_test(void);

void led_init(void)
{
    gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, GPIO_LED, GPIO_OUTPUT | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_set(gpio, GPIO_LED);
}

void spi_gpio_init(void)
{
    struct bflb_device_s *gpio;

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

void spi_init(void)
{
    struct bflb_spi_config_s spi_cfg = {
        .freq = 10 * 1000 * 1000,
        .role = SPI_ROLE_MASTER,
        .mode = SPI_MODE3,
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

void spi_test(void)
{
    uint32_t *p_tx = (uint32_t *)tx_buff;
    uint32_t *p_rx = (uint32_t *)rx_buff;

    /* data init */
    for (uint16_t i = 0; i < BUFF_LEN / 4; i++) {
        p_tx[i] = i;
        p_rx[i] = 0;
    }

    /* exchange data */
    bflb_spi_poll_exchange(spi0, p_tx, p_rx, BUFF_LEN);
}

void systick_isr()
{
    static uint32_t tick = 0;
    tick++;
    //printf("tick:%d\r\n", tick);
}

int main(void)
{
    board_init();
    //bflb_mtimer_config(1000000, systick_isr);

    led_init();
    spi_gpio_init();
    spi_init();

    cdc_acm_init();

    while (1) {
        cdc_acm_data_send_with_dtr_test();
        bflb_mtimer_delay_ms(1000);
        bflb_gpio_set(gpio, GPIO_LED);
        bflb_mtimer_delay_ms(1000);
        bflb_gpio_reset(gpio, GPIO_LED);
        spi_test();
    }
}
