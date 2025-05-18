#include <FreeRTOS.h>
#include "semphr.h"
#include "board.h"
#include "bflb_gpio.h"
#include "bflb_spi.h"
#include "bflb_mtimer.h"
#include "gw1n.h"
#include "spi_fifo.h"

/* LWIP */
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "netif/ethernet.h"
#include "ethernetif.h"

#include "ping.h"
#include "tcp_socket.h"

#define DBG_TAG "MAIN"
#include "log.h"

#define SHELL_THREAD_PRIO (configMAX_PRIORITIES - 4)
#include "shell.h"

struct netif gnetif;

#define PING_TEST
#define TCP_SERVER_CLIENT_TEST

/* This program is running on the Sipeed M0S dock only */
#define BOOT_PIN GPIO_PIN_2
static struct bflb_device_s *gpio;

static struct bflb_device_s *uart0;
extern void shell_init_with_task(struct bflb_device_s *shell);

static void gpio_button_init(void)
{
    gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, BOOT_PIN, GPIO_INPUT | GPIO_SMT_EN | GPIO_DRV_0);
}

void netif_setup_cb(void *arg) {
    struct netif *netif = (struct netif *)arg;
    LOG_I("netif_setup_cb...\r\n");
    netif_set_default(netif);
    /* tell LwIP the link is physically up */
    netif_set_link_up(netif);
    /* Bring interface up (IP stack level) */
    netif_set_up(netif);

#if defined(MCU_MODULE_B)

#if defined(PING_TEST)
    /* ---- insert static ARP entry here ---- */
    ip_addr_t target_ip;
    struct eth_addr eth;
    IP4_ADDR(&target_ip, 192,168,1,2);
    eth.addr[0] = 0x00;
    eth.addr[1] = 0x15;
    eth.addr[2] = 0x5D;
    eth.addr[3] = 0x25;
    eth.addr[4] = 0xA7;
    eth.addr[5] = 0x85;
    etharp_add_static_entry(&target_ip, &eth);

    /* init icmp */
    ping_init();
#endif /* PING_TEST */

#else  /* MCU_MODULE_A */
    /* ---- insert static ARP entry here ---- */
    ip_addr_t target_ip;
    struct eth_addr eth;
    IP4_ADDR(&target_ip, 192,168,1,3);
    eth.addr[0] = 0x00;
    eth.addr[1] = 0x15;
    eth.addr[2] = 0x5D;
    eth.addr[3] = 0x25;
    eth.addr[4] = 0xA7;
    eth.addr[5] = 0x86;
    etharp_add_static_entry(&target_ip, &eth);

#if defined(TCP_SERVER_CLIENT_TEST)
    tcp_server_task_init();
#endif

#endif /* MCU_MODULE_B */

}

void network_init(void)
{
    ip4_addr_t ipaddr, netmask, gw;

    LOG_I("set static ip address...\r\n");

#if defined(MCU_MODULE_A)
    IP4_ADDR(&ipaddr, 192, 168, 1, 2);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gw, 192, 168, 1, 1);
#else
    IP4_ADDR(&ipaddr, 192, 168, 1, 3);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gw, 192, 168, 1, 1);
#endif

    LOG_I("tcpip_init...\r\n");
    tcpip_init(NULL, NULL);

    LOG_I("netif_add...\r\n");
    netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);
    //netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

    tcpip_callback(netif_setup_cb, &gnetif);
}

int main(void)
{
    configASSERT((configMAX_PRIORITIES > 4));

    board_init();
    gpio_button_init();

#if defined(MCU_MODULE_A)
    LOG_I("Module A: start program...\r\n");
#if defined(FPGA_MODULE_GW1N_LV1)
    /* configure fpga */
    gowin_fpga_config();
    bflb_mtimer_delay_ms(200);
#endif
#else /* MCU_MODULE_A */
    LOG_I("Module B: start program...\r\n");
#endif /* MCU_MODULE_A */

    LOG_I("press button to run demo...\r\n");
    /* wait for user press start button */
    while(!bflb_gpio_read(gpio, BOOT_PIN));
    while(bflb_gpio_read(gpio, BOOT_PIN));

    /* init uart shell */
    uart0 = bflb_device_get_by_name("uart0");
    shell_init_with_task(uart0);

    LOG_I("network init...\r\n");
    network_init();

    vTaskStartScheduler();

    while (1) {
    }
}

#if defined(MCU_MODULE_B)
extern void spi_ctrl_cmd_reset_fifo(void);
int cmd_reset_spi_fifo(int argc, char **argv)
{
    printf("reset spi fifo...\r\n");
    spi_ctrl_cmd_reset_fifo();
    return 0;
}
SHELL_CMD_EXPORT_ALIAS(cmd_reset_spi_fifo, reset_spi_fifo, reset spi fifo);

#if defined(PING_TEST)
int cmd_ping_test(int argc, char **argv)
{
    printf("ping test task init\r\n");
    ping_task_init();
    return 0;
}
SHELL_CMD_EXPORT_ALIAS(cmd_ping_test, ping_test, ping test);
#endif

#if defined(TCP_SERVER_CLIENT_TEST)
int cmd_tcp_client_test(int argc, char **argv)
{
    tcp_client_task_init();
    return 0;
}
SHELL_CMD_EXPORT_ALIAS(cmd_tcp_client_test, tcp_client_test, tcp client test);
#endif

#endif
