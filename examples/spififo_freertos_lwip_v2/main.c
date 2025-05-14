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

/* ICMP test */
#include "lwip/raw.h"
#include "lwip/etharp.h"
#include "lwip/icmp.h"
#include "lwip/ip.h"
#include "lwip/inet_chksum.h"

#define DBG_TAG "MAIN"
#include "log.h"

struct netif gnetif;
struct raw_pcb *ping_pcb;

/* This program is running on the Sipeed M0S dock only */
#define BOOT_PIN GPIO_PIN_2
static struct bflb_device_s *gpio;

static void gpio_button_init(void)
{
    gpio = bflb_device_get_by_name("gpio");
    bflb_gpio_init(gpio, BOOT_PIN, GPIO_INPUT | GPIO_SMT_EN | GPIO_DRV_0);
}

static u8_t ping_recv(void *arg, struct raw_pcb *pcb, struct pbuf *p, const ip_addr_t *addr) {
    struct icmp_echo_hdr *iecho;

    if (p->len >= (PBUF_IP_HLEN + sizeof(struct icmp_echo_hdr))) {
        iecho = (struct icmp_echo_hdr *)((u8_t *)p->payload + PBUF_IP_HLEN);

        if (iecho->type == ICMP_ER && lwip_ntohs(iecho->id) == 0xABCD) {
            LOG_I("Ping reply from %s: seq=%d\r\n", ipaddr_ntoa(addr), lwip_ntohs(iecho->seqno));
            pbuf_free(p);
            return 1;
        }

        // Still respond to incoming ping requests
        if (iecho->type == ICMP_ECHO) {
            iecho->type = ICMP_ER;
            iecho->chksum += ~htons(ICMP_ECHO << 8) & 0xffff;
            iecho->chksum += htons(ICMP_ER << 8);
            p->payload = (u8_t *)p->payload + PBUF_IP_HLEN;
            raw_sendto(pcb, p, addr);
            pbuf_free(p);
            return 1;
        }
    }

    return 0;
}

void ping_init(void) {
    ping_pcb = raw_new(IP_PROTO_ICMP);
    raw_recv(ping_pcb, ping_recv, NULL);
    raw_bind(ping_pcb, IP_ADDR_ANY);
}

void send_ping(ip_addr_t *target_ip) {
    if (!ping_pcb) return;

    struct pbuf *p;
    struct icmp_echo_hdr *iecho;
    const char *payload_data = "PING!";
    size_t ping_size = sizeof(struct icmp_echo_hdr) + strlen(payload_data);

    p = pbuf_alloc(PBUF_IP, ping_size, PBUF_RAM);
    if (!p) return;

    iecho = (struct icmp_echo_hdr *)p->payload;
    ICMPH_TYPE_SET(iecho, ICMP_ECHO);
    ICMPH_CODE_SET(iecho, 0);
    iecho->chksum = 0;
    iecho->id = lwip_htons(0xABCD);
    iecho->seqno = lwip_htons(1);
    memcpy((u8_t *)iecho + sizeof(struct icmp_echo_hdr), payload_data, strlen(payload_data));
    iecho->chksum = inet_chksum(iecho, ping_size);

    raw_sendto(ping_pcb, p, target_ip);
    pbuf_free(p);
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
    //LOG_I("call send_ping...\r\n");
    //send_ping(&target_ip);    
#endif    
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
    tcpip_callback(netif_setup_cb, &gnetif);
}

#if defined(MCU_MODULE_B)
void ping_task(void *arg) {
  ip_addr_t tgt;
  IP4_ADDR(&tgt, 192, 168 , 1 ,2);
  for (;;) {
    LOG_I("Send ping...\r\n");  
    send_ping(&tgt);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void ping_task_init(void)
{
    LOG_I("[OS] Starting ping_task...\r\n");    
    xTaskCreate(ping_task, "ping_task", 1024, NULL, configMAX_PRIORITIES - 3, NULL);    
}
#endif

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

    LOG_I("network init...\r\n");

    network_init();

#if defined(MCU_MODULE_B)
    ping_task_init();
#endif

    vTaskStartScheduler();

    while (1) {
    }
}
