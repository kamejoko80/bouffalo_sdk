#include "bl616_glb.h"
#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "usbd_core.h"
#include "bflb_mtimer.h"
#include "board.h"
#include "spi_fifo_test.h"

/* This program is running on the Sipeed M0S dock only */
#define GPIO_LED   GPIO_PIN_27
#define BOOT_PIN   GPIO_PIN_2

static struct bflb_device_s *gpio;

void gpio_led_init(void)
{
    gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, GPIO_LED, GPIO_OUTPUT | GPIO_SMT_EN | GPIO_DRV_0);
    bflb_gpio_set(gpio, GPIO_LED);

    bflb_gpio_init(gpio, BOOT_PIN, GPIO_INPUT | GPIO_SMT_EN | GPIO_DRV_0);
}

int main(void)
{
    board_init();
    gpio_led_init();
    spi_fifo_interface_bus_init();

    /* Wait for fpga power is stable */
    bflb_mtimer_delay_ms(200);

#if 0
    while(1)
    {
        while(!bflb_gpio_read(gpio, BOOT_PIN));
        while(bflb_gpio_read(gpio, BOOT_PIN));
        printf("Sending cmd 0x06\r\n");
        spi_ctrl_send_byte(0x06);

        while(!bflb_gpio_read(gpio, BOOT_PIN));
        while(bflb_gpio_read(gpio, BOOT_PIN));
        printf("Sending cmd 0x01\r\n");
        spi_ctrl_send_byte(0x01);
    }
#endif    
    
#if 1
    while(1)
    {
        printf("Press button to read GW version\r\n");
        while(!bflb_gpio_read(gpio, BOOT_PIN));
        while(bflb_gpio_read(gpio, BOOT_PIN));
        spi_ctrl_cmd_read_gw_version();
        spi_ctrl_cmd_read_chip_id();
        //spi_ctrl_cmd_reset_fifo();
        spi_ctrl_cmd_write_data_len(512);
        spi_ctrl_cmd_read_data_len();
        spi_ctrl_cmd_write_data();
        spi_ctrl_cmd_read_tx_fifo_level();
        spi_ctrl_cmd_read_rx_fifo_level();
        spi_ctrl_cmd_read_data();
    }
#endif

    while (1) {
        /* Check if user press boot pin */
        if(bflb_gpio_read(gpio, BOOT_PIN))
        {
            /* wait for button release */
            while(bflb_gpio_read(gpio, BOOT_PIN));
            printf("System will reset after 3s\r\n");

            for(int i = 0; i < 3; i++)
            {
                bflb_mtimer_delay_ms(500);
                bflb_gpio_reset(gpio, GPIO_LED);
                bflb_mtimer_delay_ms(500);
                bflb_gpio_set(gpio, GPIO_LED);
            }

            bflb_gpio_reset(gpio, GPIO_LED);
            printf("System reset!\r\n");
            GLB_SW_System_Reset();
        }
    }
}
