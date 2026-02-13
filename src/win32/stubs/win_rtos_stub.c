#ifdef WINDOWS

#include "../../new_common.h"

#ifndef LINUX
#include <windows.h>
#include <timeapi.h>
#define GETSOCKETERRNO() (WSAGetLastError())
#else
#include <time.h>
#include <pthread.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
#define timeGetTime() time(NULL)
#define DWORD unsigned int
#define GETSOCKETERRNO() (errno)
#endif

// ---- RTOS thread stubs (simulator) ----
// These are only used by the WINDOWS+LINUX simulator builds in CI.
// They intentionally do not emulate priorities/stack sizes.

#ifdef LINUX
typedef struct obk_thread_start_s {
	beken_thread_function_t fn;
	beken_thread_arg_t arg;
} obk_thread_start_t;

static void* obk_thread_trampoline(void* p) {
	obk_thread_start_t* s = (obk_thread_start_t*)p;
	if (s != NULL) {
		if (s->fn != NULL) {
			s->fn(s->arg);
		}
		free(s);
	}
	return NULL;
}
#endif

OSStatus rtos_create_thread(beken_thread_t* thread,
							uint8_t priority, const char* name,
							beken_thread_function_t function,
							uint32_t stack_size, beken_thread_arg_t arg) {
	(void)priority;
	(void)name;
	(void)stack_size;

#ifndef LINUX
	HANDLE h = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)function, arg, 0, NULL);
	if (thread != NULL) {
		*thread = (beken_thread_t)h;
	}
	return kNoErr;
#else
	pthread_t* h = (pthread_t*)malloc(sizeof(pthread_t));
	if (h == NULL) {
		return -1;
	}
	obk_thread_start_t* s = (obk_thread_start_t*)malloc(sizeof(obk_thread_start_t));
	if (s == NULL) {
		free(h);
		return -1;
	}
	s->fn = function;
	s->arg = arg;

	if (pthread_create(h, NULL, obk_thread_trampoline, s) != 0) {
		free(s);
		free(h);
		return -1;
	}
	// Detach so we don't need to join.
	pthread_detach(*h);

	if (thread != NULL) {
		*thread = (beken_thread_t)h;
	}
	return kNoErr;
#endif
}

OSStatus rtos_delete_thread(beken_thread_t* thread) {
#ifndef LINUX
	// If thread==NULL treat as "delete current thread"
	if (thread == NULL) {
		ExitThread(0);
		return kNoErr;
	}
	if (*thread != NULL) {
		CloseHandle((HANDLE)(*thread));
		*thread = NULL;
	}
	return kNoErr;
#else
	if (thread == NULL) {
		pthread_exit(NULL);
		return kNoErr;
	}
	if (*thread != NULL) {
		// We detached at creation; just free the heap-handle wrapper.
		free((void*)(*thread));
		*thread = NULL;
	}
	return kNoErr;
#endif
}

OSStatus rtos_suspend_thread(beken_thread_t* thread) {
	(void)thread;
#ifndef LINUX
	ExitThread(0);
#else
	pthread_exit(NULL);
#endif
	return kNoErr;
}

// lwIP helper used by some network code in simulator.
int lwip_fcntl(int s, int cmd, int val) {
	(void)cmd;
	(void)val;

#ifndef LINUX
	u_long argp = 1;
	if (ioctlsocket(s, FIONBIO, &argp) == SOCKET_ERROR) {
		printf("ioctlsocket() error %d\n", GETSOCKETERRNO());
		return 1;
	}
	return 0;
#else
	int argp = 1;
	if (ioctlsocket(s, FIONBIO, &argp) == SOCKET_ERROR) {
		printf("ioctlsocket() error %d\n", GETSOCKETERRNO());
		return 1;
	}
	return 0;
#endif
}

#endif
