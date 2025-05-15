#include <string.h>
#include <FreeRTOS.h>
#include "semphr.h"
#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "lwip/tcpip.h"
#include "netif/ethernet.h"

#include "lwip/pbuf.h"
#include "lwip/etharp.h"        /* ETHTYPE_ARP, ETHTYPE_IP, struct eth_hdr */
#include "lwip/prot/ethernet.h" /* SIZEOF_ETH_HDR */
#include "lwip/prot/ip4.h"      /* IP_HLEN */

#include "spi_fifo.h"
#include "ethernetif.h"

#define DBG_TAG  "ETH_IF"
#include "log.h"

#define OUTPUT_BUFF_SIZE_MAX (1024)

extern uint8_t *p_tx;
extern uint8_t *p_rx;
extern void ethernetif_input_task_init(void *pvParameters);

void dump_eth_frame(struct pbuf *p) {
    if (!p || p->len < SIZEOF_ETH_HDR) {
        LOG_W("Invalid Ethernet frame\r\n");
        return;
    }

    /* 1) Flatten the pbuf chain into a contiguous buffer */
    size_t total_len = p->tot_len;
    uint8_t *buf = malloc(total_len);
    if (!buf) {
        LOG_W("Out of memory for frame dump\r\n");
        return;
    }
    pbuf_copy_partial(p, buf, total_len, 0);

    /* 2) Ethernet header */
    struct eth_hdr *eth = (struct eth_hdr *)buf;
    uint16_t ethertype = ntohs(eth->type);

    LOG_I("====== Ethernet Frame Dump ======\r\n");
    LOG_I("Dest MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
          eth->dest.addr[0], eth->dest.addr[1], eth->dest.addr[2],
          eth->dest.addr[3], eth->dest.addr[4], eth->dest.addr[5]);
    LOG_I("Src  MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
          eth->src.addr[0], eth->src.addr[1], eth->src.addr[2],
          eth->src.addr[3], eth->src.addr[4], eth->src.addr[5]);
    LOG_I("Ethertype: 0x%04X\r\n", ethertype);

    /* 3) Hex-dump of payload */
    size_t payload_len = total_len - SIZEOF_ETH_HDR;
    LOG_I("Payload (%u bytes) hex:\r\n", (unsigned)payload_len);
    for (size_t i = 0; i < payload_len; i++) {
        printf("%02X ", buf[SIZEOF_ETH_HDR + i]);
        if ((i + 1) % 16 == 0) printf("\r\n");
    }
    if (payload_len % 16) printf("\r\n");

    /* 4) Protocol-specific parsing */
    if (ethertype == ETHTYPE_ARP && payload_len >= 28) {
        uint8_t *a = buf + SIZEOF_ETH_HDR;
        uint16_t htype  = (a[0] << 8) | a[1];
        uint16_t ptype  = (a[2] << 8) | a[3];
        uint8_t  hlen   = a[4];
        uint8_t  plen   = a[5];
        uint16_t opcode = (a[6] << 8) | a[7];
        uint8_t *sha    = a + 8;
        uint8_t *spa    = sha + hlen;
        uint8_t *tha    = spa + plen;
        uint8_t *tpa    = tha + hlen;

        LOG_I("ARP: htype=0x%04X proto=0x%04X hlen=%u plen=%u opcode=%u\r\n",
              htype, ptype, hlen, plen, opcode);
        LOG_I("    Sender MAC: %02X:%02X:%02X:%02X:%02X:%02X, IP: %u.%u.%u.%u\r\n",
              sha[0],sha[1],sha[2],sha[3],sha[4],sha[5],
              spa[0],spa[1],spa[2],spa[3]);
        LOG_I("    Target MAC: %02X:%02X:%02X:%02X:%02X:%02X, IP: %u.%u.%u.%u\r\n",
              tha[0],tha[1],tha[2],tha[3],tha[4],tha[5],
              tpa[0],tpa[1],tpa[2],tpa[3]);
    }
    else if (ethertype == ETHTYPE_IP && payload_len >= IP_HLEN) {
        uint8_t *ip = buf + SIZEOF_ETH_HDR;
        uint8_t ver  = ip[0] >> 4;
        uint8_t ihl  = ip[0] & 0x0F;
        uint16_t iplen = (ip[2] << 8) | ip[3];
        uint8_t ttl  = ip[8];
        uint8_t proto= ip[9];
        uint8_t *src = ip + 12;
        uint8_t *dst = ip + 16;

        LOG_I("IP: ver=%u hl=%u proto=%u ttl=%u len=%u\r\n",
              ver, ihl, proto, ttl, iplen);
        LOG_I("    Src IP: %u.%u.%u.%u  Dst IP: %u.%u.%u.%u\r\n",
              src[0],src[1],src[2],src[3],
              dst[0],dst[1],dst[2],dst[3]);

        if (proto == 1 && payload_len >= ihl*4 + 8) {  /* ICMP */
            uint8_t *icmp = ip + ihl*4;
            uint8_t type = icmp[0], code = icmp[1];
            uint16_t id  = (icmp[4]<<8) | icmp[5];
            uint16_t seq = (icmp[6]<<8) | icmp[7];
            LOG_I("ICMP: type=%u code=%u id=%u seq=%u\r\n",
                  type, code, id, seq);
        }
        else if (proto == 6 && payload_len >= ihl*4 + 20) { /* TCP */
            uint8_t *tcp = ip + ihl*4;
            uint16_t sport = (tcp[0]<<8)|tcp[1];
            uint16_t dport = (tcp[2]<<8)|tcp[3];
            uint32_t seqn = (tcp[4]<<24)|(tcp[5]<<16)|(tcp[6]<<8)|tcp[7];
            uint32_t ackn = (tcp[8]<<24)|(tcp[9]<<16)|(tcp[10]<<8)|tcp[11];
            uint8_t flags = tcp[13];
            LOG_I("TCP: sport=%u dport=%u seq=%u ack=%u flags=0x%02X\r\n",
                  sport, dport, seqn, ackn, flags);
        }
        else if (proto == 17 && payload_len >= ihl*4 + 8) { /* UDP */
            uint8_t *udp = ip + ihl*4;
            uint16_t sport = (udp[0]<<8)|udp[1];
            uint16_t dport = (udp[2]<<8)|udp[3];
            uint16_t udplen= (udp[4]<<8)|udp[5];
            LOG_I("UDP: sport=%u dport=%u len=%u\r\n",
                  sport, dport, udplen);
        }
        else {
            LOG_I("Other IP proto: %u\r\n", proto);
        }
    }

    LOG_I("=================================\r\n");
    free(buf);
}

void low_level_init(struct netif *netif)
{
    LOG_I("low_level_init\r\n");

    /* wait for fpga configured */
    spi_fifo_interface_bus_init(32);

    /* Set netif link flag */
    netif->flags |= NETIF_FLAG_LINK_UP;

#if LWIP_ARP || LWIP_ETHERNET

    /* set MAC hardware address length */
    netif->hwaddr_len = ETH_HWADDR_LEN;

    /* set MAC hardware address */
#if defined(MCU_MODULE_A)
    netif->hwaddr[0] =  0x00;
    netif->hwaddr[1] =  0x15;
    netif->hwaddr[2] =  0x5D;
    netif->hwaddr[3] =  0x25;
    netif->hwaddr[4] =  0xA7;
    netif->hwaddr[5] =  0x85;
#else
    netif->hwaddr[0] =  0x00;
    netif->hwaddr[1] =  0x15;
    netif->hwaddr[2] =  0x5D;
    netif->hwaddr[3] =  0x25;
    netif->hwaddr[4] =  0xA7;
    netif->hwaddr[5] =  0x86;
#endif

    /* maximum transfer unit */
    netif->mtu = 1500;

#if LWIP_ARP
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
#else
    netif->flags |= NETIF_FLAG_BROADCAST;
#endif

#endif

    ethernetif_input_task_init(netif);
}

/* Workaround for Bouffalo SDK interrupt issue */
extern volatile uint8_t module_a_first_isr;

err_t low_level_output(struct netif *netif, struct pbuf *p) {
    struct pbuf *q;
    uint16_t bytes_available;
    uint16_t offset = 0;
    uint8_t *data_ptr;

    //LOG_I("low_level_output\r\n");

#if defined(MCU_MODULE_A)
    /*
     * Workaround for Bouffalo SDK interrupt issue:
     *
     * With FreeRTOS, interrupt works only if it has been initialized in a task/thread context
     * If an external GPIO interrupt has been triggered before initializing interrupt so the
     * gpio isr will be never called event there are some further interrupt triggers on the gpio.
     *
     * In this demo, after configuring FPGA, module A runs first and LWIP send an ARP message but at this time
     * module B has not yet initialized interrupt, that caused gpio isr will be never called after that
     *
     * Workaround: module A can send message after module B is ready (interrupt has been already initialized)
     *
     */
    if(module_a_first_isr == 0) {
        return ERR_OK;
    }
#endif

    // dump_eth_frame(p);

    q = p;
    while (q != NULL) {
        data_ptr = (uint8_t *)q->payload;
        uint16_t remaining = q->len;

        while (remaining > 0) {
            // Check available FIFO space
            bytes_available = spi_fifo_find_max_data_len();
            //LOG_I("bytes_available %d\r\n", bytes_available);

            if (bytes_available == 0) {
                LOG_I("wait for spi tx fifo free\r\n");
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            // Send the minimum of remaining data or available buffer space
            uint16_t to_send = (remaining < bytes_available) ? remaining : bytes_available;

            //LOG_I("spi_fifo_write\r\n");
            spi_fifo_write(data_ptr + offset, to_send);

            remaining -= to_send;
            offset += to_send;
        }

        // Move to next pbuf in chain
        q = q->next;
        offset = 0; // Reset offset for new buffer
    }

    return ERR_OK;
}

struct pbuf *low_level_input(struct netif *netif)
{
    uint16_t len;
    struct pbuf *p = NULL;
    struct pbuf *q = NULL;
    uint16_t offset;

    /* spi fifo data receive */
    len = spi_fifo_read();
    LOG_I("receive len = %d\r\n", len);

    if (len > 0) {
        /* We allocate a pbuf chain of pbufs from the Lwip buffer pool */
        p = pbuf_alloc(PBUF_LINK, len, PBUF_POOL);
    } else {
        LOG_I("error! cannot allocate pbuf\r\n");
        return NULL;
    }

    if (p != NULL) {
        offset = 4; /* Check spi_fifo read frame */
        for (q = p; q != NULL; q = q->next) {
            memcpy(q->payload, &p_rx[offset], q->len);
            offset += q->len;
        }
    }

    return p;
}

err_t ethernetif_init(struct netif *netif)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
    netif->hostname = "lwip";
#endif

    netif->name[0] = 'b';
    netif->name[1] = 'f';

#if LWIP_IPV4
#if LWIP_ARP || LWIP_ETHERNET
#if LWIP_ARP
    netif->output = etharp_output;
#else
    netif->output = low_level_output_arp_off;
#endif
#endif
#endif

    netif->linkoutput = low_level_output;

    low_level_init(netif);
    return ERR_OK;
}

void ethernetif_input(void *pvParameters)
{
    struct pbuf *p;
    struct netif *netif = (struct netif *) pvParameters;

    //LOG_I("ethernetif_input task enter\r\n");

    spi_ctrl_cmd_read_gw_version();
    spi_ctrl_cmd_read_chip_id();

    for (;;) {
        /* wait for the rdata_valid notification */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        //LOG_I("eth rx evt\r\n");

        /* small delay to let the SPI FIFO fill if needed */
        //vTaskDelay(pdMS_TO_TICKS(20));

        /* drain *all* available frames, then go back to waiting */
        while (rxdata_valid()) {
            //LOG_I("rxdata_valid\r\n");
            LOCK_TCPIP_CORE();
            p = low_level_input(netif);
            if (p != NULL) {
                //dump_eth_frame(p);
                if (ethernet_input(p, netif) != ERR_OK) {
                    pbuf_free(p);
                }
            }
            UNLOCK_TCPIP_CORE();
        }
    }

    vTaskDelete(NULL);
}