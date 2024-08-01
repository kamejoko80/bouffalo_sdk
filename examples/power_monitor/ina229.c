#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "bflb_mtimer.h"
#include "board.h"
#include "tca9534.h"
#include "ina229.h"
#include <stdio.h>
#include <string.h>

#define MAX_REG_VALUE_SIZE (40/8 + 1)

static struct bflb_device_s *spi0;
static struct bflb_device_s *gpio;

extern void cdc_acm_printf(const char *format, ...);

void ina229_reg_read(uint8_t addr, uint8_t *value, uint8_t len);
void ina229_reg_write(uint8_t addr, uint16_t value);

static void gpio0_isr(uint8_t pin)
{
    uint8_t diag_alrt[3];
    if (pin == GPIO_PIN_0) {
        ina229_reg_read(DIAG_ALRT, diag_alrt, 3);
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

    /* configure alert as external interrupt gpio */
    bflb_irq_disable(gpio->irq_num);
    bflb_gpio_init(gpio, GPIO_PIN_0, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN);
    bflb_gpio_int_init(gpio, GPIO_PIN_0, GPIO_INT_TRIG_MODE_SYNC_RISING_EDGE);
    bflb_gpio_irq_attach(GPIO_PIN_0, gpio0_isr);
}

void ina229_enable_alert_interrupt(void)
{
    bflb_irq_enable(gpio->irq_num);
}

void ina229_disable_alert_interrupt(void)
{
    bflb_irq_disable(gpio->irq_num);
}

void ina229_enable_volt_measurement(void)
{
    tca9534_pin_control(VOL_MEASURE, 1);
}

void ina229_disable_volt_measurement(void)
{
    tca9534_pin_control(VOL_MEASURE, 0);
}

static void spi_init(uint8_t baudmhz)
{
    struct bflb_spi_config_s spi_cfg = {
        .freq = baudmhz * 1000 * 1000,
        .role = SPI_ROLE_MASTER,
        .mode = SPI_MODE1,
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

void ina229_interface_bus_init(void)
{
    spi_gpio_init();
    spi_init(10);
}

void ina229_reg_read(uint8_t addr, uint8_t *value, uint8_t len)
{
    uint8_t p_tx[MAX_REG_VALUE_SIZE];

    memset(p_tx, 0, MAX_REG_VALUE_SIZE);
    memset(value, 0, len);

    p_tx[0] = (addr << 2) | 0x01;

    if(len < MAX_REG_VALUE_SIZE)
    {
        bflb_spi_poll_exchange(spi0, p_tx, value, len);
    }
    else
    {
        bflb_spi_poll_exchange(spi0, p_tx, value, MAX_REG_VALUE_SIZE);
    }
}

void ina229_reg_write(uint8_t addr, uint16_t value)
{
    uint8_t p_tx[3],  p_rx[3];

    p_tx[0] = (addr << 2);
    p_tx[1] = (uint8_t)(value >> 8);
    p_tx[2] = (uint8_t)(value & 0xFF);

    bflb_spi_poll_exchange(spi0, p_tx, p_rx, 3);
}

void ina229_init(void)
{
    uint8_t man_id[3], device_id[3], config[3];

    ina229_interface_bus_init();

    ina229_reg_read(MANUFACTURER_ID, man_id, 3);
    ina229_reg_read(DEVICE_ID, device_id, 3);

    cdc_acm_printf("Manufacturer ID = %x%x\r\n", man_id[1], man_id[2]);
    cdc_acm_printf("Device ID       = %x%x\r\n", device_id[1], device_id[2]);

    //ina229_reg_write(CONFIG, 0x7FFF);
    //ina229_reg_read(CONFIG, config, 3);
    //cdc_acm_printf("Config Read     = %x%x\r\n", config[1], config[2]);

    /* Reset the ina229 */
    ina229_reg_write(ADC_CONFIG, (0x1 << 15));
    bflb_mtimer_delay_ms(200);

    /*
     * ALATCH[15]
     * CNVR[14]: 1h = Enables conversion ready flag on ALERT pin
     * APOL[12]: 0h = Normal (Active-low, open-drain)
     */
    ina229_reg_write(DIAG_ALRT, (0x1 << 14) | (0x0 << 12));

    /* enable alert interrupt */
    ina229_enable_alert_interrupt();

    /*
     * MODE[15:12]:  9h => Continuous bus voltage only
     * VBUSCT[11:9]: (doesn't works at 50, 84, 150µs)
     *               3h => 280µs
     *               4h => 540µs
     *               5h => 1052µs
     *               6h => 2074µs
     *               7h => 4120µs
     */
    ina229_reg_write(ADC_CONFIG, (0x9 << 12)|(0x3 << 9));
}