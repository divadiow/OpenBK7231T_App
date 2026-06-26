#include "../new_common.h"
#include "../obk_config.h"

#if NEW_TCP_SERVER

#include "lwip/sockets.h"
#include "lwip/ip_addr.h"
#include "lwip/inet.h"
#include <string.h>
#include "../logging/logging.h"
#include "../hal/hal_ota.h"
#include "new_http.h"
#if PLATFORM_ESP8266
#define MAX_SOCKETS_TCP 2
#endif
#if PLATFORM_ESP8266
#define REPLY_BUFFER_SIZE			1024
#define INCOMING_BUFFER_SIZE		1024
#define HTTP_CLIENT_STACK_SIZE		4096
#define HTTP_CLIENT_STACK_SIZE		6144
#endif
#if PLATFORM_OPL1000
/*
 * The OPL1000 A2 patch image has very little free RAM once the vendor Wi-Fi,
 * supplicant and controller tasks are running. Keep the first real-port HTTP
 * server small and avoid starting the listener until the STA path has DHCP.
 */
#define MAX_SOCKETS_TCP            2
#define REPLY_BUFFER_SIZE          1024
#define INCOMING_BUFFER_SIZE       768
#define HTTP_CLIENT_STACK_SIZE     1024
#endif
#ifndef MAX_SOCKETS_TCP
#define MAX_SOCKETS_TCP MEMP_NUM_TCP_PCB
#endif

void HTTPServer_Start();

#define HTTP_SERVER_PORT			80
#define INVALID_SOCK				-1

#ifndef REPLY_BUFFER_SIZE
#define REPLY_BUFFER_SIZE			2048
#endif
#ifndef INCOMING_BUFFER_SIZE
#define INCOMING_BUFFER_SIZE		2048
#endif
#ifndef HTTP_CLIENT_STACK_SIZE
#define HTTP_CLIENT_STACK_SIZE		8192
#endif
typedef struct
{
	int fd;
	beken_thread_t thread;
	bool isCompleted;
} tcp_thread_t;

static beken_thread_t g_http_thread = NULL;
static const size_t max_socks = MAX_SOCKETS_TCP - 1;
static int listen_sock = INVALID_SOCK;
static tcp_thread_t sock[MAX_SOCKETS_TCP - 1] =
{
	[0 ... MAX_SOCKETS_TCP - 2] = { -1, NULL, false },
};

static void tcp_client_thread(tcp_thread_t* arg)
{
	int fd = arg->fd;
	char* buf = NULL;
	char* reply = NULL;
	int replyBufferSize = REPLY_BUFFER_SIZE;

	reply = (char*)os_malloc(replyBufferSize);
	buf = (char*)os_malloc(INCOMING_BUFFER_SIZE);

	if(buf == 0 || reply == 0)
	{
		ADDLOG_ERROR(LOG_FEATURE_HTTP, "TCP Client failed to malloc buffer");
		goto exit;
	}
	http_request_t request;
	memset(&request, 0, sizeof(request));

	request.fd = fd;
	request.received = buf;
	request.receivedLenmax = INCOMING_BUFFER_SIZE - 2;
	request.responseCode = HTTP_RESPONSE_OK;
	request.receivedLen = 0;
	while(1)
	{
		int remaining = request.receivedLenmax - request.receivedLen;
		int received = recv(fd, request.received + request.receivedLen, remaining, 0);
		if(received <= 0)
		{
			break;
		}
		request.receivedLen += received;
		request.received[request.receivedLen] = 0;
		if(received < remaining)
		{
			break;
		}
		// grow by INCOMING_BUFFER_SIZE
		request.receivedLenmax += INCOMING_BUFFER_SIZE;
		char *newbuf = (char*)realloc(request.received, request.receivedLenmax + 2);
		if(newbuf == NULL)
		{
			ADDLOG_ERROR(LOG_FEATURE_HTTP, "TCP Client realloc failed");
			// no memory
			//goto exit;
			request.receivedLenmax -= INCOMING_BUFFER_SIZE;
			continue;
		}
		request.received = buf = newbuf;
	}
	request.received[request.receivedLen] = 0;

	request.reply = reply;
	request.replylen = 0;
	reply[0] = '\0';

	request.replymaxlen = replyBufferSize - 1;

	if(request.receivedLen <= 0)
	{
		ADDLOG_ERROR(LOG_FEATURE_HTTP, "TCP Client is disconnected, fd: %d", fd);
		goto exit;
	}

	//addLog( "TCP received string %s",buf );
	// returns length to be sent if any
	 // ADDLOG_DEBUG(LOG_FEATURE_HTTP,  "TCP will process packet of len %i", request.receivedLen );
	int lenret = HTTP_ProcessPacket(&request);
	if(lenret > 0)
	{
		ADDLOG_DEBUG(LOG_FEATURE_HTTP, "TCP sending reply len %i", lenret);
		send(fd, reply, lenret, 0);
	}

exit:
	if(buf != NULL)
		os_free(buf);
	if(reply != NULL)
		os_free(reply);

	lwip_close(fd);
	arg->isCompleted = true;
#if PLATFORM_RDA5981
	arg->thread = NULL;
	arg->isCompleted = false;
	arg->fd = INVALID_SOCK;
	rtos_delete_thread(NULL);
#else
	rtos_suspend_thread(NULL);
#endif
}


#if PLATFORM_OPL1000
#include "../hal/hal_wifi.h"
#include "../cmnds/cmd_public.h"
extern size_t xPortGetFreeHeapSize(void);

#define OPENOPL1000_SHM_DATA   __attribute__((section(".shm_data"), used, aligned(4)))
#define OPL1000_HTTP_TRACE     0

/*
 * v37b builds on v36 and the proven v35/v36 split-M3 layout:
 *   usable application-owned SHM tail: 0x80000400 .. 0x80003fff
 *   avoided vendor/IPC-owned bottom:   0x80000000 .. 0x800003ff
 *
 * Keep the transport path synchronous and single-client, but run the stock
 * OpenBeken HTTP parser.  postany() streams replies when this scratch buffer
 * fills, so OPL1000 does not need a huge per-client reply allocation.
 */
#define OPL1000_HTTP_REQ_SIZE    1152
#define OPL1000_HTTP_REPLY_SIZE  1024

/* Keep the live OPL1000 HTTP working set in the split-M3 SHM tail. */
static char g_opl1000_http_req[OPL1000_HTTP_REQ_SIZE] OPENOPL1000_SHM_DATA = {0};
static char g_opl1000_http_reply[OPL1000_HTTP_REPLY_SIZE] OPENOPL1000_SHM_DATA = {0};
static http_request_t g_opl1000_http_request OPENOPL1000_SHM_DATA = {0};
static struct sockaddr_storage g_opl1000_source_addr OPENOPL1000_SHM_DATA = {0};

static void tcp_client_process_sync(tcp_thread_t* arg)
{
	int fd = arg->fd;
	char *buf = g_opl1000_http_req;
	http_request_t *request = &g_opl1000_http_request;
	int lenret = 0;

	memset(request, 0, sizeof(*request));
	memset(buf, 0, OPL1000_HTTP_REQ_SIZE);
	memset(g_opl1000_http_reply, 0, OPL1000_HTTP_REPLY_SIZE);

	request->fd = fd;
	request->received = buf;
	request->receivedLenmax = OPL1000_HTTP_REQ_SIZE - 2;
	request->responseCode = HTTP_RESPONSE_OK;
	request->receivedLen = 0;
	request->reply = g_opl1000_http_reply;
	request->replymaxlen = OPL1000_HTTP_REPLY_SIZE - 1;
	request->replylen = 0;

	while(1)
	{
		int remaining = request->receivedLenmax - request->receivedLen;
		int received = recv(fd, request->received + request->receivedLen, remaining, 0);
		if(received <= 0)
		{
			break;
		}
		request->receivedLen += received;
		request->received[request->receivedLen] = 0;
		if(strstr(request->received, "\r\n\r\n") != NULL ||
			strstr(request->received, "\n\n") != NULL ||
			received < remaining ||
			request->receivedLen >= request->receivedLenmax)
		{
			break;
		}
	}

	if(request->receivedLen <= 0)
	{
#if OPL1000_HTTP_TRACE
		ADDLOG_INFO(LOG_FEATURE_HTTP,
			"OPL1000 empty client connection closed, fd %d heap %u",
			fd,
			(unsigned int)xPortGetFreeHeapSize());
#endif
		goto exit;
	}

#if OPL1000_HTTP_TRACE
	ADDLOG_INFO(LOG_FEATURE_HTTP,
		"OPL1000 full OBK request len %i heap %u",
		request->receivedLen,
		(unsigned int)xPortGetFreeHeapSize());
#endif

	lenret = HTTP_ProcessPacket(request);
	if(lenret > 0 && request->replylen > 0)
	{
		send(fd, request->reply, request->replylen, 0);
		request->replylen = 0;
	}
#if OPL1000_HTTP_TRACE
	ADDLOG_INFO(LOG_FEATURE_HTTP, "OPL1000 full OBK reply len %i heap %u", lenret, (unsigned int)xPortGetFreeHeapSize());
#endif

exit:
	if(fd != INVALID_SOCK)
	{
		lwip_close(fd);
	}
	arg->fd = INVALID_SOCK;
	arg->thread = NULL;
	arg->isCompleted = false;
}

#endif

static inline char* get_clientaddr(struct sockaddr_storage* source_addr)
{
	static char address_str[128];
	char* res = NULL;
	// Convert ip address to string
	if(source_addr->ss_family == PF_INET)
	{
		res = inet_ntoa_r(((struct sockaddr_in*)source_addr)->sin_addr, address_str, sizeof(address_str) - 1);
	}
#if LWIP_IPV6
	else if(source_addr->ss_family == PF_INET6)
	{
		res = inet6_ntoa_r(((struct sockaddr_in6*)source_addr)->sin6_addr, address_str, sizeof(address_str) - 1);
	}
#endif
	if(!res)
	{
		address_str[0] = '\0';
	}
	return address_str;
}

void HTTPServer_Stop(void* arg)
{
	if(g_http_thread != NULL)
	{
		rtos_delete_thread(&g_http_thread);
	}
	if(listen_sock != INVALID_SOCK)
	{
		close(listen_sock);
	}

	for(int i = 0; i < max_socks; ++i)
	{
		if(sock[i].thread != NULL)
		{
			rtos_delete_thread(&sock[i].thread);
			sock[i].thread = NULL;
		}
		if(sock[i].fd != INVALID_SOCK)
		{
			close(sock[i].fd);
			sock[i].fd = INVALID_SOCK;
		}
	}
}

void restart_tcp_server(void* arg)
{
	HTTPServer_Start();
	rtos_delete_thread(NULL);
}

static void tcp_server_thread(beken_thread_arg_t arg)
{
	OSStatus err = kNoErr;
	int reuse = 1;

#if PLATFORM_OPL1000
	/*
	 * OBK starts the HTTP task during Main_Init_After_Delay, but on OPL1000
	 * the vendor lwIP/socket layer is not usable until the STA event path has
	 * reached GOT_IP. If we create the socket early it fails repeatedly and the
	 * restart loop burns scarce heap.
	 */
	while(!Main_IsConnectedToWiFi())
	{
		rtos_delay_milliseconds(500);
	}
	rtos_delay_milliseconds(500);
#endif

	struct sockaddr_in server_addr =
	{
		.sin_family = AF_INET,
		.sin_addr =
		{
			.s_addr = INADDR_ANY,
		},
		.sin_port = htons(HTTP_SERVER_PORT),
	};

	if(listen_sock != INVALID_SOCK) close(listen_sock);

	for(int i = 0; i < max_socks; ++i)
	{
		if(sock[i].fd != INVALID_SOCK)
		{
			close(sock[i].fd);
		}
		if(sock[i].thread != NULL)
		{
			rtos_delete_thread(&sock[i].thread);
		}
		sock[i].fd = INVALID_SOCK;
		sock[i].thread = NULL;
		sock[i].isCompleted = false;
	}

	listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(listen_sock < 0)
	{
		ADDLOG_ERROR(LOG_FEATURE_HTTP, "Unable to create socket");
		goto error;
	}

	setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));

	err = bind(listen_sock, (struct sockaddr*)&server_addr, sizeof(server_addr));
	if(err != 0)
	{
		ADDLOG_ERROR(LOG_FEATURE_HTTP, "Socket unable to bind");
		goto error;
	}
	ADDLOG_EXTRADEBUG(LOG_FEATURE_HTTP, "Socket bound on 0.0.0.0:%i", HTTP_SERVER_PORT);

#if PLATFORM_OPL1000
	err = listen(listen_sock, 1);
#else
	err = listen(listen_sock, 0);
#endif
	if(err != 0)
	{
		ADDLOG_ERROR(LOG_FEATURE_HTTP, "Error occurred during listen");
		goto error;
	}
	ADDLOG_INFO(LOG_FEATURE_HTTP, "TCP server listening");
	while(true)
	{
#if PLATFORM_OPL1000
		struct sockaddr_storage *source_addr_ptr = &g_opl1000_source_addr;
		socklen_t addr_len = sizeof(g_opl1000_source_addr);
#else
		struct sockaddr_storage source_addr;
		socklen_t addr_len = sizeof(source_addr);
#endif

		int new_idx = 0;
		for(int i = 0; i < max_socks; ++i)
		{
			if(sock[i].isCompleted)
			{
				if(sock[i].thread != NULL)
				{
					rtos_delete_thread(&sock[i].thread);
					sock[i].thread = NULL;
				}
				sock[i].isCompleted = false;
				sock[i].fd = INVALID_SOCK;
			}
		}
		for(new_idx = 0; new_idx < max_socks; ++new_idx)
		{
			if(sock[new_idx].fd == INVALID_SOCK)
			{
				break;
			}
		}
		if(new_idx < max_socks)
		{
#if PLATFORM_OPL1000
			sock[new_idx].fd = accept(listen_sock, (struct sockaddr*)source_addr_ptr, &addr_len);
#else
			sock[new_idx].fd = accept(listen_sock, (struct sockaddr*)&source_addr, &addr_len);

#if LWIP_SO_RCVTIMEO
#if LWIP_SO_SNDRCVTIMEO_NONSTANDARD
			int tv = 30 * 1000;
#else
			struct timeval tv;
			tv.tv_sec = 30;
			tv.tv_usec = 0;
#endif
			setsockopt(sock[new_idx].fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
#endif
#endif

			if(sock[new_idx].fd < 0)
			{
#if PLATFORM_OPL1000
				/* Do not enter the restart path on transient accept behaviour.  The
				 * OPL1000 socket layer/browser interaction can report short-lived
				 * errors while clients are opening/closing speculative connections.
				 */
				if(errno != EWOULDBLOCK)
				{
					ADDLOG_ERROR(LOG_FEATURE_HTTP, "OPL1000 accept failed err %i", errno);
				}
				sock[new_idx].fd = INVALID_SOCK;
				rtos_delay_milliseconds(50);
				continue;
#else
				switch(errno)
				{
					//case EAGAIN:
					case EWOULDBLOCK: break;
					case EBADF: goto error;
					default:
						ADDLOG_ERROR(LOG_FEATURE_HTTP, "[sock=%d]: Error when accepting connection, err: %i", sock[new_idx].fd, errno);
						break;
				}
#endif
			}
			else
			{
#if PLATFORM_OPL1000
#if LWIP_SO_RCVTIMEO
#if LWIP_SO_SNDRCVTIMEO_NONSTANDARD
				int tv = 3 * 1000;
#else
				struct timeval tv;
				tv.tv_sec = 3;
				tv.tv_usec = 0;
#endif
				setsockopt(sock[new_idx].fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
#endif
				//ADDLOG_EXTRADEBUG(LOG_FEATURE_HTTP, "[sock=%d]: Connection accepted from IP:%s", sock[new_idx].fd, get_clientaddr(source_addr_ptr));
#if OPL1000_HTTP_TRACE
				ADDLOG_INFO(LOG_FEATURE_HTTP, "[sock=%d]: OPL1000 sync accepted from IP:%s", sock[new_idx].fd, get_clientaddr(source_addr_ptr));
#endif
				tcp_client_process_sync(&sock[new_idx]);
#else
				rtos_delay_milliseconds(20);
				if(kNoErr != rtos_create_thread(&sock[new_idx].thread,
					BEKEN_APPLICATION_PRIORITY,
					"HTTP Client",
					(beken_thread_function_t)tcp_client_thread,
					HTTP_CLIENT_STACK_SIZE,
					(beken_thread_arg_t)&sock[new_idx]))
				{
					ADDLOG_ERROR(LOG_FEATURE_HTTP, "[sock=%d]: TCP Client thread creation failed!", sock[new_idx].fd);
					lwip_close(sock[new_idx].fd);
					sock[new_idx].fd = INVALID_SOCK;
					goto error;
				}
#endif
			}
		}
		rtos_delay_milliseconds(10);
	}

error:
	ADDLOG_ERROR(LOG_FEATURE_HTTP, "TCP Server Error");
	if(listen_sock != INVALID_SOCK)
	{
		close(listen_sock);
	}

	for(int i = 0; i < max_socks; ++i)
	{
		if(sock[i].thread != NULL)
		{
			rtos_delete_thread(&sock[i].thread);
			sock[i].thread = NULL;
		}
		if(sock[i].fd != INVALID_SOCK)
		{
			close(sock[i].fd);
			sock[i].fd = INVALID_SOCK;
		}
	}
#if PLATFORM_OPL1000
	/* Do not create a restart loop on OPL1000.  A real fatal listener error should
	 * leave a clear log and stop the HTTP task rather than eating heap/stack until
	 * the watchdog resets the chip.
	 */
	g_http_thread = NULL;
	rtos_delete_thread(NULL);
#else
	rtos_delay_milliseconds(2000);
	rtos_create_thread(NULL, BEKEN_APPLICATION_PRIORITY,
		"TCP Restart",
		(beken_thread_function_t)restart_tcp_server,
		2048,
		(beken_thread_arg_t)0);
#endif
}

void HTTPServer_Start()
{
	OSStatus err = kNoErr;

	if(g_http_thread != NULL)
	{
		rtos_delete_thread(&g_http_thread);
	}

	err = rtos_create_thread(&g_http_thread, BEKEN_APPLICATION_PRIORITY,
		"TCP_server",
		(beken_thread_function_t)tcp_server_thread,
#if PLATFORM_OPL1000
		0xC00,
#else
		0x800,
#endif
		(beken_thread_arg_t)0);
	if(err != kNoErr)
	{
		ADDLOG_ERROR(LOG_FEATURE_HTTP, "create \"TCP_server\" thread failed with %i!", err);
	}
}

#endif
