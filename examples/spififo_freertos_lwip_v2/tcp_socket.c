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
#define SEND_BUFFER_SIZE     2048      /* bytes per send() */

#define TCP_CLIENT_PRIO      (DEFAULT_THREAD_PRIO-2)  // = 3
#define TCP_SERVER_PRIO      (DEFAULT_THREAD_PRIO-2)  // = 3

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
            //LOG_I("client sent! %d\r\n", sys_now() - start);
            vTaskDelay(pdMS_TO_TICKS(1)); /* If client sends to fast then sever rx will die */
            int ret = send(sock, buf, SEND_BUFFER_SIZE, 0);
            if (ret < 0) break;
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
    xTaskCreate(tcp_server_task, "tcp_srv", 2048, NULL, TCP_SERVER_PRIO, NULL);
}

void tcp_client_task_init(void)
{
    static ip_addr_t server_ip;

    LOG_I("tcp_client_task_init\r\n");
    IP4_ADDR(&server_ip, 192,168,1,2);
    xTaskCreate(tcp_client_task, "tcp_cli", 2048, &server_ip, TCP_CLIENT_PRIO, NULL);
}
