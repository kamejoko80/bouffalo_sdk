#include "bl616_glb.h"
#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "bflb_i2c.h"
#include "usbd_core.h"
#include "bflb_mtimer.h"
#include "board.h"
#include "ina229.h"
#include "tca9534.h"
#include "gw1n.h"

#include <stdarg.h>  // For va_list

#define GPIO_LED GPIO_PIN_3
#define BOOT_PIN GPIO_PIN_2

static struct bflb_device_s *gpio;

extern void cdc_acm_init(void);
extern void cdc_acm_printf(const char *format, ...);
extern void spi_fifo_interface_bus_init(void);
extern void sspi_test(uint8_t cmd);
extern void spi_ctrl_cmd_read_gw_version(void);
extern void spi_ctrl_cmd_read_chip_id(void);
extern void spi_ctrl_send_byte(uint8_t byte);
void spi_ctrl_cmd_reset_fifo(void);
void spi_ctrl_cmd_write_data_len(uint16_t len);
void spi_ctrl_cmd_read_data_len(void);
void spi_ctrl_cmd_write_data(void);
void spi_ctrl_cmd_read_data(void);
void spi_ctrl_cmd_read_tx_fifo_level(void);
void spi_ctrl_cmd_read_rx_fifo_level(void);
void spi_ctrl_cmd_write_data_with_given_data_len(void);
void spi_ctrl_data_receive_loop(void);

void gpio_init(void)
{
    gpio = bflb_device_get_by_name("gpio");

    /* status led */
    bflb_gpio_init(gpio, GPIO_LED, GPIO_OUTPUT | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_set(gpio, GPIO_LED);

    /* boot button */
    bflb_gpio_init(gpio, BOOT_PIN, GPIO_INPUT | GPIO_SMT_EN | GPIO_DRV_0);

    /* i2c0_scl */
    bflb_gpio_init(gpio, GPIO_PIN_16, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    /* i2c0_sda */
    bflb_gpio_init(gpio, GPIO_PIN_17, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
}

int main(void)
{
    board_init();
    gpio_init();
    tca9534_init();

    /* initialize usb cdc acm */
    cdc_acm_init();

    cdc_acm_printf("Press button to start\r\n");

    /* Wait for user press boot button */
    while(!bflb_gpio_read(gpio, BOOT_PIN));
    while(bflb_gpio_read(gpio, BOOT_PIN));

    cdc_acm_printf("Start program...\r\n");
    bflb_gpio_reset(gpio, GPIO_LED);

    cdc_acm_printf("Power on FPGA\r\n");
    gowin_power_off();
    gowin_power_on();
    bflb_mtimer_delay_ms(200);
    gowin_fpga_config();

    bflb_mtimer_delay_ms(200);
    cdc_acm_printf("Start SPI FIFO test\r\n");
    spi_fifo_interface_bus_init();

#if 0
    /* press button to break */
    uint8_t i = 0;
    while(!bflb_gpio_read(gpio, BOOT_PIN))
    {
        sspi_test(i++);
        bflb_mtimer_delay_ms(50);
    }
#endif

#if 0
    while(1)
    {
        while(!bflb_gpio_read(gpio, BOOT_PIN));
        while(bflb_gpio_read(gpio, BOOT_PIN));
        cdc_acm_printf("Sending cmd 0x06\r\n");
        spi_ctrl_send_byte(0x06);

        while(!bflb_gpio_read(gpio, BOOT_PIN));
        while(bflb_gpio_read(gpio, BOOT_PIN));
        cdc_acm_printf("Sending cmd 0x01\r\n");
        spi_ctrl_send_byte(0x01);
    }
#endif

#if 0
    while(1)
    {
        cdc_acm_printf("Press button to read GW version\r\n");
        while(!bflb_gpio_read(gpio, BOOT_PIN));
        while(bflb_gpio_read(gpio, BOOT_PIN));
        spi_ctrl_cmd_read_gw_version();
        spi_ctrl_cmd_read_chip_id();
        //spi_ctrl_cmd_reset_fifo();
        spi_ctrl_cmd_write_data_len(4);
        spi_ctrl_cmd_read_data_len();
        spi_ctrl_cmd_write_data();
        spi_ctrl_cmd_read_tx_fifo_level();
        spi_ctrl_cmd_read_rx_fifo_level();
        spi_ctrl_cmd_read_data();
    }
#endif

#if 0
    while(1)
    {
        cdc_acm_printf("Press button to read GW version\r\n");
        while(!bflb_gpio_read(gpio, BOOT_PIN));
        while(bflb_gpio_read(gpio, BOOT_PIN));
        spi_ctrl_cmd_read_gw_version();
        spi_ctrl_cmd_read_chip_id();
        spi_ctrl_cmd_read_data_len();
        spi_ctrl_cmd_write_data_with_given_data_len();
        spi_ctrl_cmd_read_tx_fifo_level();
        spi_ctrl_cmd_read_rx_fifo_level();
        spi_ctrl_cmd_read_data();
    }
#endif

#if 1 /* ping pong test */
    cdc_acm_printf("Press button to read GW version\r\n");
    while(!bflb_gpio_read(gpio, BOOT_PIN));
    while(bflb_gpio_read(gpio, BOOT_PIN));
    spi_ctrl_cmd_read_gw_version();
    spi_ctrl_cmd_read_chip_id();
    spi_ctrl_data_receive_loop();
#endif

    /* turn off the led */
    bflb_gpio_set(gpio, GPIO_LED);

    while (1) {
        /* Check if user press boot pin */
        if(bflb_gpio_read(gpio, BOOT_PIN))
        {
            /* wait for button release */
            while(bflb_gpio_read(gpio, BOOT_PIN));
            cdc_acm_printf("System will reset after 3s\r\n");

            for(int i = 0; i < 3; i++)
            {
                bflb_mtimer_delay_ms(500);
                bflb_gpio_reset(gpio, GPIO_LED);
                bflb_mtimer_delay_ms(500);
                bflb_gpio_set(gpio, GPIO_LED);
            }

            bflb_gpio_reset(gpio, GPIO_LED);
            cdc_acm_printf("System reset!\r\n");
            GLB_SW_System_Reset();
        }
    }
}
