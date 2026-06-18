#include "../new_common.h"
#include "../obk_config.h"

#if NEW_TCP_SERVER

#include "lwip/sockets.h"
#include "lwip/ip_addr.h"
#include "lwip/inet.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
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

/*
 * v28 keeps the proven v25 micro-HTTP path, but moves the small request/reply
 * scratch buffers out of the normal M3 patch .bss and into the 16 KB shared
 * memory window demonstrated by the SDK at 0x80000000.
 *
 * This deliberately does NOT call vPortHeapRegionInit(). v27b proved that the
 * region is writable, but also proved that initialising it as the FreeRTOS heap
 * at the wrong time can replace/poison the normal heap. v27c proved the late
 * call is not additive. Here we treat 0x80000000 as a private manual scratch
 * arena for the single-client OPL1000 micro HTTP path only.
 */
#define OPL1000_MICRO_REQ_SIZE    512
#define OPL1000_MICRO_REPLY_SIZE  768
#define OPL1000_STATUS_BODY_SIZE  320
#define OPL1000_PAGE_BODY_SIZE    640
#define OPL1000_CMD_SIZE          128

#define OPL1000_SHM_SCRATCH_BASE      0x80000000u
#define OPL1000_SHM_REQ_OFFSET        0x0000u
#define OPL1000_SHM_REPLY_OFFSET      0x0200u
#define OPL1000_SHM_STATUS_OFFSET     0x0500u
#define OPL1000_SHM_PAGE_OFFSET       0x0640u
#define OPL1000_SHM_CMD_OFFSET        0x08C0u
#define OPL1000_SHM_CMD_ESC_OFFSET    0x0940u
#define OPL1000_SHM_USED_SIZE         0x09C0u

#define g_opl1000_micro_req       ((char *)(uintptr_t)(OPL1000_SHM_SCRATCH_BASE + OPL1000_SHM_REQ_OFFSET))
#define g_opl1000_micro_reply     ((char *)(uintptr_t)(OPL1000_SHM_SCRATCH_BASE + OPL1000_SHM_REPLY_OFFSET))
#define g_opl1000_status_body     ((char *)(uintptr_t)(OPL1000_SHM_SCRATCH_BASE + OPL1000_SHM_STATUS_OFFSET))
#define g_opl1000_page_body       ((char *)(uintptr_t)(OPL1000_SHM_SCRATCH_BASE + OPL1000_SHM_PAGE_OFFSET))
#define g_opl1000_cmd             ((char *)(uintptr_t)(OPL1000_SHM_SCRATCH_BASE + OPL1000_SHM_CMD_OFFSET))
#define g_opl1000_cmd_escaped     ((char *)(uintptr_t)(OPL1000_SHM_SCRATCH_BASE + OPL1000_SHM_CMD_ESC_OFFSET))

static http_request_t g_opl1000_request_probe;
static struct sockaddr_storage g_opl1000_source_addr;
static bool g_opl1000_shm_scratch_probed = false;

static void opl1000_shm_scratch_probe(const char *where)
{
	if(g_opl1000_shm_scratch_probed)
	{
		return;
	}
	g_opl1000_shm_scratch_probed = true;

	memset(g_opl1000_micro_req, 0, OPL1000_SHM_USED_SIZE);
	g_opl1000_micro_req[0] = (char)0x28;
	g_opl1000_cmd_escaped[OPL1000_CMD_SIZE - 1] = (char)0x82;
	ADDLOG_INFO(LOG_FEATURE_HTTP,
		"OPL1000 v28 SHM scratch %s base=0x%08x used=0x%x first=0x%02x last=0x%02x heap=%u",
		where,
		(unsigned int)OPL1000_SHM_SCRATCH_BASE,
		(unsigned int)OPL1000_SHM_USED_SIZE,
		(unsigned int)(unsigned char)g_opl1000_micro_req[0],
		(unsigned int)(unsigned char)g_opl1000_cmd_escaped[OPL1000_CMD_SIZE - 1],
		(unsigned int)xPortGetFreeHeapSize());
	memset(g_opl1000_micro_req, 0, OPL1000_SHM_USED_SIZE);
}

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

static int opl1000_hex_nibble(char c)
{
	if(c >= '0' && c <= '9') return c - '0';
	if(c >= 'a' && c <= 'f') return c - 'a' + 10;
	if(c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}

static int opl1000_url_decode_token(const char *src, char *dst, int dstMax)
{
	int out = 0;
	if(dstMax <= 0)
	{
		return 0;
	}
	while(*src && *src != ' ' && *src != '&' && *src != '\r' && *src != '\n' && out < (dstMax - 1))
	{
		if(*src == '+')
		{
			dst[out++] = ' ';
			src++;
		}
		else if(*src == '%' && src[1] && src[2])
		{
			int hi = opl1000_hex_nibble(src[1]);
			int lo = opl1000_hex_nibble(src[2]);
			if(hi >= 0 && lo >= 0)
			{
				dst[out++] = (char)((hi << 4) | lo);
				src += 3;
			}
			else
			{
				dst[out++] = *src++;
			}
		}
		else
		{
			dst[out++] = *src++;
		}
	}
	dst[out] = 0;
	return out;
}

static bool opl1000_extract_cmnd(const char *buf, char *cmd, int cmdMax)
{
	const char *p = strstr(buf, "cmnd=");
	if(p == NULL)
	{
		cmd[0] = 0;
		return false;
	}
	p += 5;
	return opl1000_url_decode_token(p, cmd, cmdMax) > 0;
}

static const char *opl1000_cmd_rc_name(commandResult_t cr)
{
	switch(cr)
	{
	case CMD_RES_OK: return "OK";
	case CMD_RES_UNKNOWN_COMMAND: return "UNKNOWN_COMMAND";
	case CMD_RES_NOT_ENOUGH_ARGUMENTS: return "NOT_ENOUGH_ARGUMENTS";
	case CMD_RES_EMPTY_STRING: return "EMPTY_STRING";
	case CMD_RES_BAD_ARGUMENT: return "BAD_ARGUMENT";
	case CMD_RES_ERROR: return "ERROR";
	default: return "UNKNOWN_RESULT";
	}
}

static void opl1000_json_escape_small(const char *src, char *dst, int dstMax)
{
	int out = 0;
	if(dstMax <= 0)
	{
		return;
	}
	while(*src && out < (dstMax - 1))
	{
		char c = *src++;
		if((c == '\"' || c == '\\') && out < (dstMax - 2))
		{
			dst[out++] = '\\';
			dst[out++] = c;
		}
		else if(c >= 32 && c < 127)
		{
			dst[out++] = c;
		}
	}
	dst[out] = 0;
}

static int opl1000_write_status(int fd, const char *httpTag)
{
	snprintf(g_opl1000_status_body, OPL1000_STATUS_BODY_SIZE,
		"{\"app\":\"OpenOPL1000\",\"ip\":\"%s\",\"heap\":%u,\"wifi\":%d,\"http\":\"%s\"}\n",
		HAL_GetMyIPString(),
		(unsigned int)xPortGetFreeHeapSize(),
		Main_IsConnectedToWiFi() ? 1 : 0,
		httpTag);
	return opl1000_http_write_response(fd,
		"200 OK",
		"application/json",
		g_opl1000_status_body);
}

static int opl1000_handle_direct_cmnd(int fd, const char *buf)
{
	commandResult_t cr;
	if(!opl1000_extract_cmnd(buf, g_opl1000_cmd, OPL1000_CMD_SIZE))
	{
		return opl1000_http_write_response(fd,
			"400 Bad Request",
			"application/json",
			"{\"error\":\"missing cmnd\"}\n");
	}

	cr = CMD_ExecuteCommand(g_opl1000_cmd, COMMAND_FLAG_SOURCE_HTTP);
	opl1000_json_escape_small(g_opl1000_cmd, g_opl1000_cmd_escaped, OPL1000_CMD_SIZE);
	snprintf(g_opl1000_status_body, OPL1000_STATUS_BODY_SIZE,
		"{\"app\":\"OpenOPL1000\",\"cmd\":\"%s\",\"cmd_rc\":%d,\"cmd_result\":\"%s\",\"heap\":%u,\"wifi\":%d}\n",
		g_opl1000_cmd_escaped,
		(int)cr,
		opl1000_cmd_rc_name(cr),
		(unsigned int)xPortGetFreeHeapSize(),
		Main_IsConnectedToWiFi() ? 1 : 0);
	return opl1000_http_write_response(fd,
		"200 OK",
		"application/json",
		g_opl1000_status_body);
}

static int opl1000_micro_fallback(int fd, const char *buf)
{
	if(strncmp(buf, "GET /favicon.ico", 16) == 0)
	{
		return opl1000_http_write_response(fd,
			"404 Not Found",
			"text/plain",
			"not found\n");
	}
	else if(strncmp(buf, "GET /cm?", 8) == 0)
	{
		return opl1000_handle_direct_cmnd(fd, buf);
	}
	else if(strncmp(buf, "GET /status", 11) == 0)
	{
		return opl1000_write_status(fd, "v28-shm-scratch");
	}
	else if(strncmp(buf, "GET /cfg", 8) == 0 ||
		strncmp(buf, "GET /index", 10) == 0)
	{
		return opl1000_http_write_response(fd,
			"503 Service Unavailable",
			"text/html",
			"<html><body><h1>OpenOPL1000</h1><p>Full OBK GUI route is disabled in v28.</p><p>Use /, /status, or /cm?cmnd=Status.</p></body></html>\n");
	}

	snprintf(g_opl1000_page_body, OPL1000_PAGE_BODY_SIZE,
		"<html><body><h1>OpenOPL1000</h1>"
		"<p>IP: %s</p>"
		"<p>Heap: %u</p>"
		"<p>v28 direct-command micro UI using SHM scratch buffers</p>"
		"<p><a href='/status'>status json</a></p>"
		"<p><a href='/cm?cmnd=Status'>Status command</a></p>"
		"<p><a href='/cm?cmnd=Power%%20Toggle'>Power Toggle</a></p>"
		"<form action='/cm' method='get'>"
		"<input name='cmnd' value='Status' style='width:220px'>"
		"<button type='submit'>Run command</button>"
		"</form>"
		"<p><small>Full GUI remains disabled until transport and heap are proven stable.</small></p>"
		"</body></html>\n",
		HAL_GetMyIPString(),
		(unsigned int)xPortGetFreeHeapSize());
	return opl1000_http_write_response(fd,
		"200 OK",
		"text/html",
		g_opl1000_page_body);
}

static void tcp_client_process_sync(tcp_thread_t* arg)
{
	int fd = arg->fd;
	char *buf = g_opl1000_micro_req;
	http_request_t *requestProbe = &g_opl1000_request_probe;
	int sendRc = 0;

	memset(requestProbe, 0, sizeof(*requestProbe));
	memset(buf, 0, OPL1000_MICRO_REQ_SIZE);

	requestProbe->fd = fd;
	requestProbe->received = buf;
	requestProbe->receivedLenmax = OPL1000_MICRO_REQ_SIZE - 2;
	requestProbe->responseCode = HTTP_RESPONSE_OK;
	requestProbe->receivedLen = 0;

	while(1)
	{
		int remaining = requestProbe->receivedLenmax - requestProbe->receivedLen;
		int received = recv(fd, requestProbe->received + requestProbe->receivedLen, remaining, 0);
		if(received <= 0)
		{
			break;
		}
		requestProbe->receivedLen += received;
		requestProbe->received[requestProbe->receivedLen] = 0;
		if(strstr(requestProbe->received, "\r\n\r\n") != NULL ||
			strstr(requestProbe->received, "\n\n") != NULL ||
			received < remaining ||
			requestProbe->receivedLen >= requestProbe->receivedLenmax)
		{
			break;
		}
	}

	if(requestProbe->receivedLen <= 0)
	{
		ADDLOG_INFO(LOG_FEATURE_HTTP,
			"OPL1000 empty client connection closed, fd %d heap %u",
			fd,
			(unsigned int)xPortGetFreeHeapSize());
		goto exit;
	}

	ADDLOG_INFO(LOG_FEATURE_HTTP,
		"OPL1000 micro request len %i heap %u",
		requestProbe->receivedLen,
		(unsigned int)xPortGetFreeHeapSize());

	sendRc = opl1000_micro_fallback(fd, requestProbe->received);
	ADDLOG_INFO(LOG_FEATURE_HTTP, "OPL1000 micro reply rc %i heap %u", sendRc, (unsigned int)xPortGetFreeHeapSize());

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
	opl1000_shm_scratch_probe("tcp_server_start");
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
