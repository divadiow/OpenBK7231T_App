#ifdef WINDOWS

#include "selftest_local.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <string.h>
#include <stdio.h>

extern int g_httpPort;

#if defined(OBK_SANITIZER_RUN)

static int SelfTest_SendAll(int s, const void* buf, int len)
{
	const char* p = (const char*)buf;
	int remaining = len;
	while (remaining > 0)
	{
		int sent = lwip_send(s, p, remaining, 0);
		if (sent <= 0)
		{
			return -1;
		}
		p += sent;
		remaining -= sent;
	}
	return 0;
}

static void SelfTest_HTTPServer_SendLargeHeader(const char* host, int port, int headerBytes)
{
	int s;
	struct sockaddr_in sa;
	char prefix[256];
	const char* suffix = "\r\n\r\n";
	char chunk[1024];

	s = lwip_socket(AF_INET, SOCK_STREAM, 0);
	SELFTEST_ASSERT(s >= 0);

	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port = htons((u16_t)port);
	sa.sin_addr.s_addr = inet_addr(host);

	SELFTEST_ASSERT(lwip_connect(s, (struct sockaddr*)&sa, sizeof(sa)) == 0);

	// Send a large single header line to force the server-side receive buffer to grow.
	// This is designed to exercise realloc/append paths under ASan/UBSan.
	snprintf(prefix, sizeof(prefix),
		"GET /cm?cmnd=POWER%%20TOGGLE HTTP/1.1\r\n"
		"Host: %s\r\n"
		"X-Fill: ",
		host);

	SELFTEST_ASSERT(SelfTest_SendAll(s, prefix, (int)strlen(prefix)) == 0);

	memset(chunk, 'A', sizeof(chunk));
	while (headerBytes > 0)
	{
		int toSend = headerBytes > (int)sizeof(chunk) ? (int)sizeof(chunk) : headerBytes;
		SELFTEST_ASSERT(SelfTest_SendAll(s, chunk, toSend) == 0);
		headerBytes -= toSend;
	}

	SELFTEST_ASSERT(SelfTest_SendAll(s, suffix, (int)strlen(suffix)) == 0);

	// Close immediately; the goal is to stress the server receive/parsing path.
	lwip_close(s);
}

#endif

void Test_HTTP_Client() {
	// reset whole device
	SIM_ClearOBK(0);


	PIN_SetPinRoleForPinIndex(9, IOR_Relay);
	PIN_SetPinChannelForPinIndex(9, 1);

	SELFTEST_ASSERT_CHANNEL(1, 0);

	// Also nice method of testing: addRepeatingEvent 2 -1 SendGet http://192.168.0.103/cm?cmnd=POWER%20TOGGLE
	///CMD_ExecuteCommand("SendGet http://192.168.0.103/cm?cmnd=POWER%20TOGGLE", 0);
	//CMD_ExecuteCommand("SendGet http://192.168.0.104/cm?cmnd=POWER%20TOGGLE", 0);
	// The following selftest does a real TCP send to our loopback 127.0.0.1 address.
	// It is enabled only for sanitizer runs (CI/local) because it depends on the HTTP server being bound.
#if defined(OBK_SANITIZER_RUN)
	{
		char cmd[192];
		printf("SAN_HTTPSTRESS_START port=%d\n", g_httpPort);

		// Basic sanity: issue a normal local request and verify it toggles channel 1.
		snprintf(cmd, sizeof(cmd), "SendGet http://127.0.0.1:%d/cm?cmnd=POWER%%20TOGGLE", g_httpPort);
		CMD_ExecuteCommand(cmd, 0);
		Sim_RunSeconds(2, true);
		SELFTEST_ASSERT_CHANNEL(1, 1);

		snprintf(cmd, sizeof(cmd), "SendGet http://127.0.0.1:%d/cm?cmnd=POWER%%20TOGGLE", g_httpPort);
		CMD_ExecuteCommand(cmd, 0);
		Sim_RunSeconds(2, true);
		SELFTEST_ASSERT_CHANNEL(1, 0);

		// Stress: send an oversized single header to force server-side buffer growth.
		SelfTest_HTTPServer_SendLargeHeader("127.0.0.1", g_httpPort, 128 * 1024);
		Sim_RunSeconds(2, true);

		printf("SAN_HTTPSTRESS_OK\n");
	}
#endif

	//CMD_ExecuteCommand("SendGet http://127.0.0.1/cm?cmnd=POWER%20TOGGLE", 0);
	//Sim_RunFrames(15, false);
	//SELFTEST_ASSERT_CHANNEL(1, 1);
}


#endif
