/*
 * mqtt_broker.c
 *
 * A minimal MQTT broker using Bouffalo Lab SDK (FreeRTOS + lwIP)
 * and the MQTT-C client library headers (mqtt.h, mqtt_pal.h).
 *
 * Ensure your build:
 *  - Adds the path to mqtt/inc/ into your -I flags
 *  - Compiles mqtt/src/mqtt.c and mqtt/src/mqtt_pal.c alongside this file
 */

#include "lwip/tcp.h"
#include "mqtt.h"
#include "mqtt_pal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

#define MQTT_PORT               1883
#define MAX_CLIENTS             5
#define MAX_TOPIC_LEN           64
#define MAX_TOPICS_PER_CLIENT   8
#define RX_BUFFER_SIZE          512

typedef struct broker_client {
    struct tcp_pcb  *pcb;
    char             topics[MAX_TOPICS_PER_CLIENT][MAX_TOPIC_LEN];
    uint8_t          topic_count;
} broker_client_t;

static broker_client_t clients[MAX_CLIENTS];
static uint8_t buf[RX_BUFFER_SIZE];

/* lwIP callbacks */
static err_t  broker_accept(void *arg, struct tcp_pcb *new_pcb, err_t err);
static err_t  broker_recv  (void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void   broker_err   (void *arg, err_t err);

/* Helpers */
static void handle_connect     (broker_client_t *c);
static void handle_subscribe   (broker_client_t *c, uint16_t packet_id);
static void handle_publish     (broker_client_t *c, const char *pub_topic, const uint8_t *payload, uint16_t payload_len);
static void handle_pingreq     (broker_client_t *c);
static void handle_disconnect  (broker_client_t *c);
static bool topic_matches(const char *filter, const char *topic);

/* Public API */
void mqtt_broker_init(void)
{
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) {
        printf("[BROKER] tcp_new() failed\r\n");
        return;
    }
    tcp_bind(pcb, IP_ADDR_ANY, MQTT_PORT);
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, broker_accept);
    printf("[BROKER] MQTT broker listening on port %d\r\n", MQTT_PORT);
}

/* New incoming TCP connection */
static err_t broker_accept(void *arg, struct tcp_pcb *new_pcb, err_t err)
{
    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(err);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].pcb == NULL) {
            broker_client_t *c = &clients[i];
            c->pcb         = new_pcb;
            c->topic_count = 0;

            tcp_arg(new_pcb, c);
            tcp_recv(new_pcb, broker_recv);
            tcp_err(new_pcb, broker_err);
            printf("[BROKER] Client connected (slot %d)\r\n", i);
            return ERR_OK;
        }
    }
    /* No slots free */
    tcp_abort(new_pcb);
    return ERR_ABRT;
}

/* Data received on TCP socket */
static err_t broker_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    broker_client_t *c = arg;
    if (!p) {
        /* connection closed */
        printf("[BROKER] Client disconnected\r\n");
        c->pcb = NULL;
        return tcp_close(tpcb);
    }

    uint8_t *data = (uint8_t*)p->payload;
    uint8_t  ctrl = data[0] >> 4;
    uint16_t rem_len = data[1];  // assumes single-byte remaining length
    uint16_t offset = 2;

    switch (ctrl) {
        case 1:   /* CONNECT */
            handle_connect(c);
            break;

        case 8: { /* SUBSCRIBE */
            uint16_t pid = (data[offset] << 8) | data[offset + 1];
            offset += 2;
            uint16_t tlen = (data[offset] << 8) | data[offset + 1];
            offset += 2;
            if (tlen < MAX_TOPIC_LEN && c->topic_count < MAX_TOPICS_PER_CLIENT) {
                memcpy(c->topics[c->topic_count], &data[offset], tlen);
                c->topics[c->topic_count][tlen] = '\0';
                printf("[BROKER] SUBSCRIBE to '%s'\r\n", c->topics[c->topic_count]);
                c->topic_count++;
            }
            handle_subscribe(c, pid);
            break;
        }

        case 3: { /* PUBLISH QoS0 */
            uint16_t tlen = (data[offset] << 8) | data[offset + 1];
            offset += 2;
            if (tlen >= MAX_TOPIC_LEN) break;
            char topic[MAX_TOPIC_LEN];
            memcpy(topic, &data[offset], tlen);
            topic[tlen] = '\0';
            offset += tlen;
            uint16_t payload_len = rem_len - (2 + tlen);
            handle_publish(c, topic, &data[offset], payload_len);
            break;
        }

        case 12: /* PINGREQ */
            handle_pingreq(c);
            break;

        case 14: /* DISCONNECT */
            handle_disconnect(c);
            break;

        default:
            /* unsupported */
            break;
    }

    pbuf_free(p);
    return ERR_OK;
}

/* TCP error */
static void broker_err(void *arg, err_t err)
{
    broker_client_t *c = arg;
    LWIP_UNUSED_ARG(err);
    printf("[BROKER] Error on client\r\n");
    c->pcb = NULL;
}

/* --------------------------------------------------------------------------
 * MQTT packet handlers & serializers
 * -------------------------------------------------------------------------- */

static void handle_connect(broker_client_t *c)
{
    uint8_t resp[] = { 0x20, 0x02, 0x00, 0x00 };  // CONNACK: success
    tcp_write(c->pcb, resp, sizeof(resp), TCP_WRITE_FLAG_COPY);
    tcp_output(c->pcb);
    printf("[BROKER] Sent CONNACK\r\n");
}

static void handle_subscribe(broker_client_t *c, uint16_t packet_id)
{
    uint8_t resp[5];
    resp[0] = 0x90;                  // SUBACK
    resp[1] = 0x03;                  // remaining len
    resp[2] = (packet_id >> 8) & 0xFF;
    resp[3] =  packet_id       & 0xFF;
    resp[4] = 0x00;                  // QoS0 granted
    tcp_write(c->pcb, resp, sizeof(resp), TCP_WRITE_FLAG_COPY);
    tcp_output(c->pcb);
    printf("[BROKER] Sent SUBACK\r\n");
}

static void handle_publish(broker_client_t *c, const char *pub_topic, const uint8_t *payload, uint16_t payload_len)
{
    for (int i = 0; i < MAX_CLIENTS; i++) {
        broker_client_t *cl = &clients[i];
        if (cl->pcb == NULL) continue;
        for (int t = 0; t < cl->topic_count; t++) {
            if (topic_matches(cl->topics[t], pub_topic)) {
                uint16_t tlen = strlen(pub_topic);
                uint16_t rem = 2 + tlen + payload_len;
                size_t   idx = 0;
                buf[idx++] = 0x30;           // PUBLISH
                buf[idx++] = rem;
                buf[idx++] = (tlen >> 8) & 0xFF;
                buf[idx++] =  tlen       & 0xFF;
                memcpy(&buf[idx], pub_topic, tlen); idx += tlen;
                memcpy(&buf[idx], payload,   payload_len);
                tcp_write(cl->pcb, buf, idx + payload_len, TCP_WRITE_FLAG_COPY);
                tcp_output(cl->pcb);
            }
        }
    }
    printf("[BROKER] Broadcast PUBLISH '%s'\r\n", pub_topic);
}

static void handle_pingreq(broker_client_t *c)
{
    uint8_t resp[] = { 0xD0, 0x00 };  // PINGRESP
    tcp_write(c->pcb, resp, sizeof(resp), TCP_WRITE_FLAG_COPY);
    tcp_output(c->pcb);
    printf("[BROKER] Sent PINGRESP\r\n");
}

static void handle_disconnect(broker_client_t *c)
{
    tcp_close(c->pcb);
    c->pcb = NULL;
    printf("[BROKER] Client DISCONNECT\r\n");
}

static bool topic_matches(const char *filter, const char *topic)
{
    while (*filter && *topic) {
        if (*filter == '#') return true;
        if (*filter == '+') {
            while (*topic && *topic != '/') topic++;
            filter++; topic++;
            continue;
        }
        if (*filter != *topic) return false;
        filter++; topic++;
    }
    return (*filter == *topic) || (*filter == '#' && filter[1] == '\0');
}

#if defined(MCU_MODULE_A)
#ifdef CONFIG_SHELL
#include <shell.h>
int cmd_mqtt_broker(int argc, const char **argv)
{
    mqtt_broker_init();
    return 0;
}

/*
 * Example:
 * > mqtt_broker
 */
SHELL_CMD_EXPORT_ALIAS(cmd_mqtt_broker, mqtt_broker, mqtt broker start);
#endif
#endif
