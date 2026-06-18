#include "../new_common.h"
#include "../obk_config.h"

#if NEW_TCP_SERVER

#include "lwip/sockets.h"
#include "lwip/ip_addr.h"
#include "lwip/inet.h"
#include <string.h>
#include <stdio.h>
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
extern size_t xPortGetFreeHeapSize(void);

/*
 * v20 proved that Wi-Fi, DHCP, TCP accept and static micro HTTP all work once
 * BLE is disabled. v22 keeps the stable single-client/static-buffer transport,
 * treats empty browser connections as normal, and only routes lightweight /cm
 * command requests through the real OpenBeken HTTP parser. Full UI pages are
 * deliberately kept behind micro fallback responses until the server lifecycle
 * is proven stable.
 *
 * Keep these buffers static: heap is valuable after Wi-Fi/lwIP, and v17 showed
 * that large locals can overflow the TCP_server stack.
 */
#define OPL1000_OBK_REQ_SIZE      768
#define OPL1000_OBK_REPLY_SIZE    1024
#define OPL1000_MICRO_REPLY_SIZE  768
#define OPL1000_STATUS_BODY_SIZE  192
#define OPL1000_PAGE_BODY_SIZE    256

static char g_opl1000_obk_req[OPL1000_OBK_REQ_SIZE];
static char g_opl1000_obk_reply[OPL1000_OBK_REPLY_SIZE];
static char g_opl1000_micro_reply[OPL1000_MICRO_REPLY_SIZE];
static char g_opl1000_status_body[OPL1000_STATUS_BODY_SIZE];
static char g_opl1000_page_body[OPL1000_PAGE_BODY_SIZE];
static http_request_t g_opl1000_request;
static struct sockaddr_storage g_opl1000_source_addr;

static int opl1000_send_all(int fd, const char *data, int len)
{
	int sentTotal = 0;
	while(sentTotal < len)
	{
		int sent = send(fd, data + sentTotal, len - sentTotal, 0);
		if(sent <= 0)
		{
			return sent;
		}
		sentTotal += sent;
	}
	return sentTotal;
}

static int opl1000_http_write_response(int fd, const char *status, const char *ctype, const char *body)
{
	int bodyLen = (int)strlen(body);
	int len = snprintf(g_opl1000_micro_reply,
		OPL1000_MICRO_REPLY_SIZE,
		"HTTP/1.1 %s\r\n"
		"Connection: close\r\n"
		"Content-Type: %s\r\n"
		"Content-Length: %d\r\n"
		"Cache-Control: no-store\r\n"
		"\r\n"
		"%s",
		status,
		ctype,
		bodyLen,
		body);

	if(len <= 0)
	{
		return len;
	}
	if(len >= OPL1000_MICRO_REPLY_SIZE)
	{
		len = OPL1000_MICRO_REPLY_SIZE - 1;
	}
	return opl1000_send_all(fd, g_opl1000_micro_reply, len);
}

static bool opl1000_is_cmd_request(const char *buf)
{
	/* First OBK target for OPL1000: command endpoint only.  The normal GUI can
	 * allocate/format substantially more data and is enabled later, once the
	 * socket lifecycle is stable under normal browser behaviour.
	 */
	return strncmp(buf, "GET /cm?", 8) == 0 ||
		strncmp(buf, "POST /cm", 8) == 0;
}

static int opl1000_micro_fallback(int fd, const char *buf, bool *handled)
{
	*handled = true;

	if(strncmp(buf, "GET /favicon.ico", 16) == 0)
	{
		return opl1000_http_write_response(fd,
			"404 Not Found",
			"text/plain",
			"not found\n");
	}
	else if(strncmp(buf, "GET / ", 6) == 0 ||
		strncmp(buf, "GET /opl1000", 12) == 0)
	{
		snprintf(g_opl1000_page_body, sizeof(g_opl1000_page_body),
			"<html><body><h1>OpenOPL1000</h1>"
			"<p>IP: %s</p>"
			"<p>Heap: %u</p>"
			"<p>v22 hardened HTTP transport build</p>"
			"<p>Try <code>/status</code> or <code>/cm?cmnd=status</code>.</p>"
			"</body></html>\n",
			HAL_GetMyIPString(),
			(unsigned int)xPortGetFreeHeapSize());
		return opl1000_http_write_response(fd,
			"200 OK",
			"text/html",
			g_opl1000_page_body);
	}
	else if(strncmp(buf, "GET /status", 11) == 0)
	{
		snprintf(g_opl1000_status_body, sizeof(g_opl1000_status_body),
			"{\"app\":\"OpenOPL1000\",\"ip\":\"%s\",\"heap\":%u,\"wifi\":%d,\"http\":\"v22\"}\n",
			HAL_GetMyIPString(),
			(unsigned int)xPortGetFreeHeapSize(),
			Main_IsConnectedToWiFi() ? 1 : 0);
		return opl1000_http_write_response(fd,
			"200 OK",
			"application/json",
			g_opl1000_status_body);
	}
	else if(strncmp(buf, "GET /cfg", 8) == 0 ||
		strncmp(buf, "GET /index", 10) == 0)
	{
		return opl1000_http_write_response(fd,
			"503 Service Unavailable",
			"text/plain",
			"OpenOPL1000 v22: full GUI route is intentionally disabled in this transport-hardening build. Try /cm?cmnd=status.\n");
	}

	*handled = false;
	return 0;
}

static void tcp_client_process_sync(tcp_thread_t* arg)
{
	int fd = arg->fd;
	http_request_t *request = &g_opl1000_request;
	int sendRc = 0;
	int lenret = 0;
	bool handled = false;

	memset(request, 0, sizeof(*request));
	memset(g_opl1000_obk_req, 0, OPL1000_OBK_REQ_SIZE);
	memset(g_opl1000_obk_reply, 0, OPL1000_OBK_REPLY_SIZE);

	request->fd = fd;
	request->received = g_opl1000_obk_req;
	request->receivedLenmax = OPL1000_OBK_REQ_SIZE - 2;
	request->responseCode = HTTP_RESPONSE_OK;
	request->receivedLen = 0;
	request->reply = g_opl1000_obk_reply;
	request->replylen = 0;
	request->replymaxlen = OPL1000_OBK_REPLY_SIZE - 1;

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
		/* Normal browsers may open speculative/keepalive sockets and close them
		 * without sending a request.  On OPL1000 this must not become a server
		 * restart path; just close the accepted socket and continue.
		 */
		ADDLOG_INFO(LOG_FEATURE_HTTP,
			"OPL1000 empty client connection closed, fd %d heap %u",
			fd,
			(unsigned int)xPortGetFreeHeapSize());
		goto exit;
	}

	ADDLOG_INFO(LOG_FEATURE_HTTP,
		"OPL1000 request len %i heap %u",
		request->receivedLen,
		(unsigned int)xPortGetFreeHeapSize());

	if(opl1000_is_cmd_request(request->received))
	{
		lenret = HTTP_ProcessPacket(request);
		if(lenret > 0)
		{
			sendRc = opl1000_send_all(fd, request->reply, lenret);
		}
		else
		{
			/* Some OBK handlers stream via postany()/send() and return 0. */
			sendRc = lenret;
		}
		ADDLOG_INFO(LOG_FEATURE_HTTP,
			"OPL1000 OBK command reply rc %i lenret %i heap %u",
			sendRc,
			lenret,
			(unsigned int)xPortGetFreeHeapSize());
	}
	else
	{
		sendRc = opl1000_micro_fallback(fd, request->received, &handled);
		if(!handled)
		{
			sendRc = opl1000_http_write_response(fd,
				"404 Not Found",
				"text/plain",
				"OpenOPL1000 v22: endpoint not enabled in hardened transport build. Try /status or /cm?cmnd=status.\n");
		}
		ADDLOG_INFO(LOG_FEATURE_HTTP,
			"OPL1000 micro reply rc %i heap %u",
			sendRc,
			(unsigned int)xPortGetFreeHeapSize());
	}

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
		struct sockaddr_storage *source_addr_ptr = &source_addr;
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
			sock[new_idx].fd = accept(listen_sock, (struct sockaddr*)source_addr_ptr, &addr_len);

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
#if LWIP_SO_RCVTIMEO
#if LWIP_SO_SNDRCVTIMEO_NONSTANDARD
#if PLATFORM_OPL1000
				int tv = 3 * 1000;
#else
				int tv = 30 * 1000;
#endif
#else
				struct timeval tv;
#if PLATFORM_OPL1000
				tv.tv_sec = 3;
#else
				tv.tv_sec = 30;
#endif
				tv.tv_usec = 0;
#endif
				setsockopt(sock[new_idx].fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
#endif
				//ADDLOG_EXTRADEBUG(LOG_FEATURE_HTTP, "[sock=%d]: Connection accepted from IP:%s", sock[new_idx].fd, get_clientaddr(source_addr_ptr));
#if PLATFORM_OPL1000
				ADDLOG_INFO(LOG_FEATURE_HTTP, "[sock=%d]: OPL1000 sync accepted from IP:%s", sock[new_idx].fd, get_clientaddr(source_addr_ptr));
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
		0x800,
		(beken_thread_arg_t)0);
	if(err != kNoErr)
	{
		ADDLOG_ERROR(LOG_FEATURE_HTTP, "create \"TCP_server\" thread failed with %i!", err);
	}
}

#endif
