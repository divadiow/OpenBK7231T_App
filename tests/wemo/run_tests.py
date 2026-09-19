#!/usr/bin/env python3
"""Build the unchanged production driver against host-only API stubs.

Run from any directory: python3 tests/wemo/run_tests.py
Requires gcc or clang. No SDK checkout or third-party Python packages required.
This does not replace the full Windows simulator or hardware interoperability tests.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

STUB = r'''
#ifndef WEMO_STUB_H
#define WEMO_STUB_H
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdbool.h>
#include <netinet/in.h>
#define CHANNEL_MAX 16
#define ChType_Toggle 1
#define MAX_HEADERS 16
#define HTTP_GET 0
#define HTTP_POST 2
#define LOG_ERROR 1
#define LOG_FEATURE_HTTP 1
#define httpMimeTypeXML "text/xml"
typedef struct { char *bodystart; int bodylen,contentLength,responseCode,numheaders; char *headers[MAX_HEADERS]; } http_request_t;
typedef int (*http_callback_fn)(http_request_t*);
int CMD_ExecuteCommand(const char*,int);
int h_isChannelRelay(int);
int CHANNEL_GetType(int);
int CHANNEL_Get(int);
void CHANNEL_Set(int,int,int);
int LED_IsLEDRunning(void);
int LED_GetEnableAll(void);
void LED_SetEnableAll(int);
void WiFI_GetMacAddress(char*);
const char *HAL_GetMyIPString(void);
const char *CFG_GetDeviceName(void);
void addLogAdv(int,int,const char*,...);
int poststr(http_request_t*,const char*);
int hprintf255(http_request_t*,const char*,...);
void poststr_escaped(http_request_t*,char*);
void http_setup(http_request_t*,const char*);
int http_rest_error(http_request_t*,int,char*);
int HTTP_RegisterCallback(const char*,int,http_callback_fn,int);
int my_strnicmp(const char*,const char*,int);
void DRV_SSDP_SendReply(struct sockaddr_in*,const char*);
void WEMO_Init(void);
void WEMO_Shutdown(void);
void *test_malloc(size_t);
#endif
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sanitize', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    compiler = os.environ.get('CC', 'gcc')
    if not shutil.which(compiler):
        parser.error(f'C compiler not found: {compiler}')
    with tempfile.TemporaryDirectory(prefix='obk-wemo-') as tmp:
        work = Path(tmp)
        driver = root / 'src/driver/drv_wemo.c'
        target = work / 'src/driver/drv_wemo.c'
        target.parent.mkdir(parents=True)
        # Byte-for-byte production source; only its dependencies are stubbed.
        shutil.copyfile(driver, target)
        (work / 'stub.h').write_text(STUB)
        for path in ['src/new_common.h', 'src/new_pins.h', 'src/new_cfg.h', 'src/cmnds/cmd_public.h',
                     'src/logging/logging.h', 'src/hal/hal_wifi.h',
                     'src/driver/drv_local.h', 'src/driver/drv_ssdp.h',
                     'src/httpserver/new_http.h']:
            f = work / path
            f.parent.mkdir(parents=True, exist_ok=True)
            f.write_text('#include "stub.h"\n')
        text = (root / 'tests/wemo/host_test.c').read_text()
        # Only the driver's malloc is intercepted for allocation-failure tests.
        text = text.replace('#include "src/driver/drv_wemo.c"',
                            '#define malloc test_malloc\n#include "src/driver/drv_wemo.c"\n#undef malloc')
        (work / 'host_test.c').write_text(text)
        flags = ['-std=c99', '-Wall', '-Wextra', '-Werror', '-g', '-O1']
        if args.sanitize:
            flags += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
        # Do not define USER_SW_VER in the API stub. Compile once without it,
        # as well as with ordinary and XML-sensitive build-version strings.
        # Expectations are independent of the production preprocessor branch.
        versions = [
            ('absent', None, ''),
            ('defined', 'test-version', 'test-version'),
            ('escaped', 'test<&>"version', 'test&lt;&amp;&gt;&quot;version'),
        ]
        for led in (0, 1):
            for label, version, expected in versions:
                exe = work / f'wemo-test-{led}-{label}'
                defines = [f'-DENABLE_LED_BASIC={led}',
                           '-DWEMO_TEST_VERSION_XML=' + json.dumps(expected)]
                if version is not None:
                    defines.append('-DUSER_SW_VER=' + json.dumps(version))
                print(f'Testing {compiler}: LED={led}, version={label}', flush=True)
                subprocess.run([compiler, *flags, *defines, '-I', str(work),
                                str(work / 'host_test.c'), '-o', str(exe)], check=True)
                subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    main()
