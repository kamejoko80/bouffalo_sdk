#include "bl616_glb.h"
#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "usbd_core.h"
#include "bflb_mtimer.h"
#include "board.h"
#include "gw1n.h"
#include "lmcxo2.h"
#include "spi_fifo_test.h"
#include "debug.h"

/* This program is running on the Sipeed M0S dock only */
#define BOOT_PIN GPIO_PIN_2

static struct bflb_device_s *gpio;

void gpio_button_init(void)
{
    gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, BOOT_PIN, GPIO_INPUT | GPIO_SMT_EN | GPIO_DRV_0);
}

int main(void)
{
    board_init();
    gpio_button_init();
    cdc_acm_init();

    /* wait for user press start button */
    while(!bflb_gpio_read(gpio, BOOT_PIN));
    while(bflb_gpio_read(gpio, BOOT_PIN));

#if defined(MCU_MODULE_A)

    usb_printf("Module A: start program...\r\n");

#if defined(FPGA_MODULE_GW1N_LV1)
    /* configure fpga */
    gowin_fpga_config();
#endif
    
#if defined(FPGA_MODULE_LMCXO2_640HC)
    /* configure fpga */
    lmcxo2_fpga_config();
    
    //extern int xsvftool_blk616_scan(void);
    //uint32_t idcode = xsvftool_blk616_scan();
    
    //usb_printf("idcode=0x%08lx, revision=0x%01lx, part=0x%04lx, manufactor=0x%03lx\r\n", idcode,
    //    (idcode >> 28) & 0xf, (idcode >> 12) & 0xffff, (idcode >> 1) & 0x7ff);

#endif    
    
    /* wait for fpga configured */
    //spi_fifo_interface_bus_init(32);    
    //bflb_mtimer_delay_ms(300);
    //spi_ctrl_cmd_read_gw_version();
    //spi_ctrl_cmd_read_chip_id();    
    
#else /* MCU_MODULE_B */

    usb_printf("Module B: start program...\r\n");

    /* Init module b spi */
    spi_fifo_interface_bus_init();

    /* Wait for fpga power is stable */
    bflb_mtimer_delay_ms(200);

#endif

#if 0 /* ping pong test */
    printf("Press button to read GW version\r\n");
    while(!bflb_gpio_read(gpio, BOOT_PIN));
    while(bflb_gpio_read(gpio, BOOT_PIN));
    spi_ctrl_cmd_read_gw_version();
    spi_ctrl_cmd_read_chip_id();
    spi_ctrl_cmd_write_data_with_given_data_len();
    spi_ctrl_data_receive_loop();
#endif

    while(1);
}
