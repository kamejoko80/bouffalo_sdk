#include <FreeRTOS.h>
#include "semphr.h"

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

/* ping */
#include "ping.h"

/* log trace */
#define DBG_TAG "PING"
#include "log.h"

static struct raw_pcb *ping_pcb;
static uint32_t ping_sent_time_ms;

/* ----------------------------------------------------------------------
 * send_ping(): record timestamp just before sending
 * -------------------------------------------------------------------- */
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
    memcpy((uint8_t *)iecho + sizeof(struct icmp_echo_hdr),
           payload_data, strlen(payload_data));
    iecho->chksum = inet_chksum(iecho, ping_size);

    /* record send time (ms since boot) */
    ping_sent_time_ms = sys_now();

    raw_sendto(ping_pcb, p, target_ip);
    pbuf_free(p);
}

/* ----------------------------------------------------------------------
 * ping_recv(): decode ICMP echo-reply and log bytes, time, TTL, seq
 * -------------------------------------------------------------------- */
static u8_t ping_recv(void *arg, struct raw_pcb *pcb,
                      struct pbuf *p, const ip_addr_t *addr)
{
    struct icmp_echo_hdr *iecho;

    /* must have IP header + echo header */
    if (p->len >= (PBUF_IP_HLEN + sizeof(struct icmp_echo_hdr))) {
        /* iecho points to the ICMP header */
        iecho = (struct icmp_echo_hdr *)
                ((uint8_t *)p->payload + PBUF_IP_HLEN);

        /* only handle our echo-replies */
        if (iecho->type == ICMP_ER &&
            lwip_ntohs(iecho->id) == 0xABCD)
        {
            /* grab the IP header to extract TTL & total length */
            struct ip_hdr *iphdr = (struct ip_hdr *)p->payload;
            uint8_t  ttl       = IPH_TTL(iphdr);
            uint8_t  ihl_bytes = IPH_HL(iphdr) * 4;
            uint16_t total_ip_len = ntohs(IPH_LEN(iphdr));
            /* bytes = total IP length minus IP header */
            uint16_t bytes = total_ip_len - ihl_bytes;
            /* round-trip time */
            uint32_t rtt = sys_now() - ping_sent_time_ms;

            LOG_I("Ping reply from %s: bytes=%u time=%lums TTL=%u seq=%u\r\n",
                  ipaddr_ntoa(addr),
                  bytes,
                  rtt,
                  ttl,
                  lwip_ntohs(iecho->seqno));

            pbuf_free(p);
            return 1;
        }

        /* still handle incoming echo *requests* if you want to reply */
        if (iecho->type == ICMP_ECHO) {
            iecho->type = ICMP_ER;
            iecho->chksum += ~htons(ICMP_ECHO << 8) & 0xffff;
            iecho->chksum += htons(ICMP_ER << 8);
            p->payload = (uint8_t *)p->payload + PBUF_IP_HLEN;
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

void ping_task(void *arg) {
  ip_addr_t tgt;
  IP4_ADDR(&tgt, 192, 168 , 1 ,2);

  for (;;) {
    LOG_I("Send ping...\r\n");
    send_ping(&tgt);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void ping_task_init(void)
{
    LOG_I("[OS] Starting ping_task...\r\n");
    xTaskCreate(ping_task, "ping_task", 1024, NULL, configMAX_PRIORITIES - 4, NULL);
}
