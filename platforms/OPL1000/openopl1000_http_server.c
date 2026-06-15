#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "openopl1000_http_server.h"
#include "sys_os_config.h"

#ifndef OPENOPL1000_HTTP_PORT
#define OPENOPL1000_HTTP_PORT 80
#endif

#define OPENOPL1000_HTTP_BACKLOG 2
#define OPENOPL1000_HTTP_STACK_SIZE 1024
#define OPENOPL1000_HTTP_RETRY_DELAY_MS 3000

static osThreadId s_httpThread;
static bool s_httpStarted;

static const char s_openOpl1000HttpResponse[] =
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: text/html; charset=utf-8\r\n"
    "Connection: close\r\n"
    "Cache-Control: no-store\r\n"
    "\r\n"
    "<!doctype html>"
    "<html>"
    "<head>"
    "<meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>OpenOPL1000</title>"
    "<style>body{font-family:sans-serif;margin:2rem;line-height:1.4}code{background:#eee;padding:.15rem .35rem}</style>"
    "</head>"
    "<body>"
    "<h1>OpenOPL1000</h1>"
    "<p>OPL1000 station and TCP/IP bring-up is running.</p>"
    "<p>This is a minimal SDK-native HTTP proof page, not the full OpenBeken web UI yet.</p>"
    "<p>Target AP: <code>test</code></p>"
    "</body>"
    "</html>";

static void OpenOPL1000_HttpServeClient(int clientSock)
{
    char request[192];
    int received;
    int sent;
    const char *response = s_openOpl1000HttpResponse;
    int responseLen = (int)strlen(response);

    received = lwip_recv(clientSock, request, sizeof(request) - 1, 0);
    if (received > 0)
    {
        request[received] = '\0';
        printf("[OpenOPL1000] HTTP request received (%d bytes)\r\n", received);
    }
    else
    {
        printf("[OpenOPL1000] HTTP request read returned %d\r\n", received);
    }

    sent = lwip_send(clientSock, response, responseLen, 0);
    printf("[OpenOPL1000] HTTP response sent=%d expected=%d\r\n", sent, responseLen);

    lwip_close(clientSock);
}

static int OpenOPL1000_HttpCreateListenSocket(void)
{
    struct sockaddr_in serverAddr;
    int listenSock;
    int reuse = 1;

    listenSock = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listenSock < 0)
    {
        printf("[OpenOPL1000] HTTP socket create failed: %d\r\n", listenSock);
        return -1;
    }

    lwip_setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_len = sizeof(serverAddr);
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = lwip_htons(OPENOPL1000_HTTP_PORT);
    serverAddr.sin_addr.s_addr = lwip_htonl(IPADDR_ANY);

    if (lwip_bind(listenSock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        printf("[OpenOPL1000] HTTP bind port %d failed\r\n", OPENOPL1000_HTTP_PORT);
        lwip_close(listenSock);
        return -1;
    }

    if (lwip_listen(listenSock, OPENOPL1000_HTTP_BACKLOG) < 0)
    {
        printf("[OpenOPL1000] HTTP listen failed\r\n");
        lwip_close(listenSock);
        return -1;
    }

    printf("[OpenOPL1000] HTTP server listening on port %d\r\n", OPENOPL1000_HTTP_PORT);
    return listenSock;
}

static void OpenOPL1000_HttpThread(void *args)
{
    int listenSock;

    (void)args;

    while (1)
    {
        listenSock = OpenOPL1000_HttpCreateListenSocket();
        if (listenSock < 0)
        {
            osDelay(OPENOPL1000_HTTP_RETRY_DELAY_MS);
            continue;
        }

        while (1)
        {
            struct sockaddr_in clientAddr;
            socklen_t clientAddrLen = sizeof(clientAddr);
            int clientSock;

            clientSock = lwip_accept(listenSock, (struct sockaddr *)&clientAddr, &clientAddrLen);
            if (clientSock < 0)
            {
                printf("[OpenOPL1000] HTTP accept failed: %d\r\n", clientSock);
                osDelay(250);
                continue;
            }

            printf("[OpenOPL1000] HTTP client connected\r\n");
            OpenOPL1000_HttpServeClient(clientSock);
        }
    }
}

void OpenOPL1000_HttpServerStart(void)
{
    osThreadDef_t threadDef;

    if (s_httpStarted)
    {
        return;
    }

    s_httpStarted = true;

    threadDef.name = "openopl1000_http";
    threadDef.stacksize = OPENOPL1000_HTTP_STACK_SIZE;
    threadDef.tpriority = OS_TASK_PRIORITY_APP;
    threadDef.instances = 0;
    threadDef.pthread = OpenOPL1000_HttpThread;

    s_httpThread = osThreadCreate(&threadDef, NULL);
    if (s_httpThread == NULL)
    {
        s_httpStarted = false;
        printf("[OpenOPL1000] HTTP task create failed\r\n");
    }
    else
    {
        printf("[OpenOPL1000] HTTP task created\r\n");
    }
}
