#include <FreeRTOS.h>
#include "semphr.h"
#include "board.h"
#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "usbd_core.h"
#include "bflb_mtimer.h"
#include "gw1n.h"
#include "spi_fifo.h"

#define DBG_TAG "MAIN"
#include "log.h"

/* This program is running on the Sipeed M0S dock only */
#define BOOT_PIN GPIO_PIN_2
static struct bflb_device_s *gpio;

static void gpio_button_init(void)
{
    gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, BOOT_PIN, GPIO_INPUT | GPIO_SMT_EN | GPIO_DRV_0);
}

int main(void)
{
    configASSERT((configMAX_PRIORITIES > 4));

    board_init();
    gpio_button_init();

    /* wait for user press start button */
    while(!bflb_gpio_read(gpio, BOOT_PIN));
    while(bflb_gpio_read(gpio, BOOT_PIN));

#if defined(MCU_MODULE_A)

    LOG_I("Module A: start program...\r\n");

#if defined(FPGA_MODULE_GW1N_LV1)
    /* configure fpga */
    gowin_fpga_config();
    /* wait for fpga configured */
    spi_fifo_interface_bus_init(32);
    bflb_mtimer_delay_ms(300);
    spi_ctrl_cmd_read_gw_version();
    spi_ctrl_cmd_read_chip_id();
    spi_fifo_task_init();
#endif

#else /* MCU_MODULE_A */

    LOG_I("Module B: start program...\r\n");
    spi_fifo_interface_bus_init(32);
    bflb_mtimer_delay_ms(300);
    spi_ctrl_cmd_read_gw_version();
    spi_ctrl_cmd_read_chip_id();
    spi_ctrl_cmd_write_data_with_given_data_len();
    spi_fifo_task_init();

#endif /* MCU_MODULE_A */

    vTaskStartScheduler();

    while (1) {
    }
}
