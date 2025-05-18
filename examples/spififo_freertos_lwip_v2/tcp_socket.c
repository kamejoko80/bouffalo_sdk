#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <lwip/sys.h>
#include <lwip/sockets.h>

#include "lwipopts_user.h"

#define DBG_TAG  "TCP_SOCKET"
#include "log.h"

#define THROUGHPUT_TCP_PORT  5001      /* TCP port for test */
#define TEST_DURATION_MS     2000      /* run each test for 2 seconds */
#define SEND_BUFFER_SIZE     4096      /* bytes per send() */

#define TCP_CLIENT_PRIO      (DEFAULT_THREAD_PRIO-3)  // = 5
#define TCP_SERVER_PRIO      (DEFAULT_THREAD_PRIO-3)  // = 5

static void tcp_server_task(void *arg) {
    int listen_sock, conn_sock;
    struct sockaddr_in addr, client;
    socklen_t client_len = sizeof(client);
    char *buf = pvPortMalloc(SEND_BUFFER_SIZE);
    if (!buf) {
        LOG_E("Server: OOM buffer\r\n");
        vTaskDelete(NULL);
        return;
    }

    listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) {
        LOG_E("Server: socket() failed\r\n");
        vPortFree(buf);
        vTaskDelete(NULL);
        return;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(THROUGHPUT_TCP_PORT);
    addr.sin_addr.s_addr = PP_HTONL(INADDR_ANY);

    bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr));
    listen(listen_sock, 1);

    for (;;) {
        LOG_I("Server: waiting for connection...\r\n");
        conn_sock = accept(listen_sock, (struct sockaddr *)&client, &client_len);
        if (conn_sock < 0) continue;

        LOG_I("Server: connection from %s\r\n", inet_ntoa(client.sin_addr));
        uint32_t start = sys_now();
        uint32_t total = 0;
        int  len;

        /* Drain until peer closes */
        while ((len = recv(conn_sock, buf, SEND_BUFFER_SIZE, 0)) > 0) {
            total += len;
        }

        uint32_t elapsed = sys_now() - start; /* ms */
        /* Mbps = (total bytes * 8) / (elapsed ms) / 1000 */
        float mbps = ((float)total * 8.0f) / ((float)elapsed * 1000.0f);
        LOG_I("Server: received %u bytes in %ums → %.2f Mbps\r\n",
              total, elapsed, mbps);

        closesocket(conn_sock);
    }
}

static void tcp_client_task(void *arg) {
    ip_addr_t *server_ip = (ip_addr_t *)arg;
    int sock;
    struct sockaddr_in addr;
    char *buf = pvPortMalloc(SEND_BUFFER_SIZE);
    if (!buf) {
        LOG_E("Client: OOM buffer\r\n");
        vTaskDelete(NULL);
        return;
    }
    /* fill with pattern */
    memset(buf, 0xA5, SEND_BUFFER_SIZE);

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(THROUGHPUT_TCP_PORT);
    addr.sin_addr.s_addr = server_ip->addr;

    for (;;) {
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            LOG_E("Client: socket() fail\r\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /************************************************************************
         * KNOWN ISSUE:
         *  1) Module A send ACK TCP/IP frame but right before module B is
         *     sending new package data.
         *  2) In module B, event there is an rxdata_valid interrupt event
         *     but SPI resource has been acquired for send() before, dead lock happened.
         *  3) Summary module B didn't read ACK frame from module B, next module A read
         *     then the tx fifo data bytes of module A are not free properly.
         ************************************************************************/

        /************************************************************************
         * THIS IS THE MAGIC FOR A WORKAROUND:
         * Add definitions in lwipopts_user.h
         *   #define TCP_MSS                       1024                         // limit each segment to 1024 bytes
         *   #define TCP_SND_BUF                   (2*TCP_MSS)                  // total send-buffer space
         *   #define TCP_SND_QUEUELEN              (4 * TCP_SND_BUF / TCP_MSS)  // number of segments in queue
         *
         ************************************************************************/

        LOG_I("Client: connecting to %s\r\n", ipaddr_ntoa(server_ip));
        if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            LOG_E("Client: connect() fail\r\n");
            closesocket(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        uint32_t start = sys_now();
        uint32_t total = 0;

        while ((sys_now() - start) < TEST_DURATION_MS) {
            /* now blocking: this send() will wait here
               until the prior SEND_BUFFER_SIZE segment is ACKed */
            int ret = send(sock, buf, SEND_BUFFER_SIZE, 0);
            if (ret < 0) {
                LOG_W("Client: send() error %d\r\n", errno);
                break;
            }
            total += ret;
        }

        uint32_t elapsed = sys_now() - start;
        float mbps = ((float)total * 8.0f) / ((float)elapsed * 1000.0f);
        LOG_I("Client: sent %u bytes in %ums → %.2f Mbps\r\n",
              total, elapsed, mbps);

        closesocket(sock);
        /* pause before next run */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void tcp_server_task_init(void)
{
    LOG_I("tcp_server_task_init\r\n");
    xTaskCreate(tcp_server_task, "tcp_srv", 4096, NULL, TCP_SERVER_PRIO, NULL);
}

void tcp_client_task_init(void)
{
    static ip_addr_t server_ip;

    LOG_I("tcp_client_task_init\r\n");
    IP4_ADDR(&server_ip, 192,168,1,2);
    xTaskCreate(tcp_client_task, "tcp_cli", 4096, &server_ip, TCP_CLIENT_PRIO, NULL);
}
