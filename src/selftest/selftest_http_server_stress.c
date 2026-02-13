#ifdef WINDOWS

#include "selftest_local.h"

#if LINUX
#include <sys/types.h>
#include <stdint.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
typedef int SIM_SOCKET_T;
#define SIM_INVALID_SOCKET (-1)
#define SIM_CLOSE_SOCKET(s) close(s)
#else
#include <stdint.h>
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET SIM_SOCKET_T;
#define SIM_INVALID_SOCKET INVALID_SOCKET
#define SIM_CLOSE_SOCKET(s) closesocket(s)
#endif

// Declared in http_tcp_server_nonblocking.c (simulator web server)
extern int g_httpPort;

static int SIM_SendAll(SIM_SOCKET_T s, const char* buf, int len) {
	int sentTotal = 0;
	while (sentTotal < len) {
#if LINUX
		int r = (int)send(s, buf + sentTotal, (size_t)(len - sentTotal), 0);
#else
		int r = send(s, buf + sentTotal, len - sentTotal, 0);
#endif
		if (r <= 0) {
			return -1;
		}
		sentTotal += r;
	}
	return 0;
}

// This test exists purely to exercise the simulator HTTP server receive-buffer growth path under ASan/UBSan.
// It sends a very large header line to the local OBK HTTP server.
// Passing condition: process does not crash and sanitizers report nothing.
void Test_HTTP_Server_LargeHeader() {
	// Ensure OBK has had time to initialise and start the web server.
	Sim_RunFrames(50, false);

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons((uint16_t)g_httpPort);
	addr.sin_addr.s_addr = htonl(0x7F000001); // 127.0.0.1

	SIM_SOCKET_T s = SIM_INVALID_SOCKET;
	int cr = -1;

	// Give the server some time to start and make the test resilient to CI timing.
	for (int attempt = 0; attempt < 50; attempt++)
	{
		if (s != SIM_INVALID_SOCKET)
		{
			SIM_CLOSE_SOCKET(s);
			s = SIM_INVALID_SOCKET;
		}
#if LINUX
		s = socket(AF_INET, SOCK_STREAM, 0);
#else
		s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
#endif
		SELFTEST_ASSERT(s != SIM_INVALID_SOCKET);

		cr = connect(s, (struct sockaddr*)&addr, sizeof(addr));
		if (cr == 0)
		{
			break;
		}
		Sim_RunFrames(10, false);
	}
	SELFTEST_ASSERT(cr == 0);

	const int fillerLen = 128 * 1024; // 128KB
	const char* prefix = "GET /cm?cmnd=STATUS%2011 HTTP/1.1\r\nHost: 127.0.0.1\r\nX-Fill: ";
	const char* suffix = "\r\n\r\n";

	int prefixLen = (int)strlen(prefix);
	int suffixLen = (int)strlen(suffix);
	int totalLen = prefixLen + fillerLen + suffixLen;

	char* req = (char*)malloc((size_t)totalLen + 1);
	SELFTEST_ASSERT(req != 0);

	memcpy(req, prefix, (size_t)prefixLen);
	memset(req + prefixLen, 'A', (size_t)fillerLen);
	memcpy(req + prefixLen + fillerLen, suffix, (size_t)suffixLen);
	req[totalLen] = 0;

	int sr = SIM_SendAll(s, req, totalLen);
	free(req);
	SELFTEST_ASSERT(sr == 0);

	// Allow the server to process the request.
	Sim_RunFrames(200, false);

	SIM_CLOSE_SOCKET(s);
}

#endif // WINDOWS
