#!/usr/bin/env python3
"""Host regression for the SV6X66 open SoftAP configuration and startup."""
import argparse
from pathlib import Path
import subprocess
import tempfile

PLATFORM = Path(__file__).resolve().parents[1]
APP = PLATFORM.parents[1]
HEADERS = ("wifi_api.h", "wificonf.h", "softap_func.h", "netstack.h", "lwip/inet.h", "lwip/ip_addr.h")
ENV = r'''#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <arpa/inet.h>
typedef struct { uint32_t addr; } ip_addr_t;
#define ipaddr_addr(text) inet_addr(text)
#define IP4_ADDR(ip,a,b,c,d) ((ip)->addr = htonl(((uint32_t)(a)<<24) | ((uint32_t)(b)<<16) | ((uint32_t)(c)<<8) | (uint32_t)(d)))
#define ip4_addr_get_u32(ip) ((ip)->addr)
typedef uint8_t u8; typedef uint16_t u16; typedef uint32_t u32;
typedef int8_t s8; typedef int16_t s16;
typedef int WIFI_OPMODE;
#define DUT_AP 2
#define DUT_STA 1
#define IF0_NAME "if0"
#define IF1_NAME "if1"
#define WRONG_PASSPHRASE 1
#define UNSUPPORT_ENCRYPT 2
#define STA_CONN 1
#define NET80211_CRYPT_UNKNOWN 0
struct WIFI_RSP { int wifistatus; int reason; };
typedef struct WIFI_RSP WIFI_RSP;
typedef struct { int stastatus; } STAINFO;
typedef struct { int run_channel; } IEEE80211STATUS;
typedef struct {
    u32 start_ip, end_ip, gw, subnet;
    s8 dfsignore, max_sta_num, encryt_mode, keylen;
    u8 key[64]; u8 channel; s16 beacon_interval;
    s8 ssid_length; s8 ssid[32]; s8 ssid_hidden;
} SOFTAP_CUSTOM_CONFIG;
int softap_set_custom_conf(SOFTAP_CUSTOM_CONFIG *);
int wifi_register_softap_cb(void (*)(STAINFO *));
int DUT_wifi_start(WIFI_OPMODE);
int get_if_config_2(char *, u8 *, u32 *, u32 *, u32 *, u32 *, u8 *, int);
int get_DUT_wifi_mode(void); int set_if_config_2(u8,u8,u32,u32,u32,u32);
int wifi_connect_active_5(u8 *,u8,u8 *,u8,u8,u8,u8 *,u8,u8,void (*)(WIFI_RSP *));
void wifi_disconnect(void (*)(WIFI_RSP *));
int get_wifi_status(void);
int get_connectap_info(u8,u8 *,u8 *,u8 *,u8,u8 *,u8 *);
extern IEEE80211STATUS gwifistatus;
'''
HARNESS = r'''#include <stdio.h>
#include <stdlib.h>
#include <string.h>
int HAL_SetupWiFiOpenAccessPoint(const char *ssid);
static SOFTAP_CUSTOM_CONFIG captured;
static int have_config, beacon_count, dhcp_rejected;
IEEE80211STATUS gwifistatus;
int softap_set_custom_conf(SOFTAP_CUSTOM_CONFIG *config) {
    captured = *config; have_config = 1; return 0;
}
int wifi_register_softap_cb(void (*callback)(STAINFO *)) { (void)callback; return 0; }
/* The SDK reports success even when its DHCP pool validation aborts AP startup. */
int DUT_wifi_start(WIFI_OPMODE mode) {
    unsigned leases;
    if (mode != DUT_AP || !have_config) return 0;
    leases = captured.end_ip - captured.start_ip;
    if (leases > (unsigned)captured.max_sta_num) { dhcp_rejected = 1; return 0; }
    ++beacon_count;
    return 0;
}
int get_if_config_2(char *name,u8 *dhcp,u32 *ip,u32 *mask,u32 *gateway,u32 *dns,u8 *mac,int maclen) {
    (void)name; (void)maclen; *dhcp=0; *ip=0xC0A80401; *mask=0xFFFFFF00;
    *gateway=0xC0A80401; *dns=0xC0A80401; memset(mac,0,6); return 0;
}
int main(void) {
    const char *ssid = "OpenBeken-Test";
    int result = HAL_SetupWiFiOpenAccessPoint(ssid);
    if (result != 0) return 1;
    if (!have_config) return 2;
    if (dhcp_rejected || beacon_count != 1) return 7;
    if (captured.start_ip != 0xC0A80402 || captured.end_ip != 0xC0A80405) return 3;
    if (captured.max_sta_num != 4 || captured.gw != 0xC0A80401 || captured.subnet != 0xFFFFFF00) return 4;
    if (captured.encryt_mode != 0 || captured.channel != 1 || captured.beacon_interval != 100) return 5;
    if (captured.ssid_length != (int)strlen(ssid) || memcmp(captured.ssid,ssid,strlen(ssid)) != 0) return 6;
    return 0;
}
'''

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--main-source", type=Path, default=PLATFORM / "../.." / "src/hal/sv6x66/hal_wifi_sv6x66.c")
    args = parser.parse_args()
    source = args.main_source.resolve()
    with tempfile.TemporaryDirectory(prefix="sv6166f-wifi-ap-") as tmp:
        temp = Path(tmp)
        stubs = temp / "stubs"
        for header in HEADERS:
            path = stubs / header
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("/* declarations supplied by test_wifi_ap_env.h */\n")
        env = stubs / "test_wifi_ap_env.h"
        env.write_text(ENV)
        harness = temp / "wifi_ap_harness.c"
        harness.write_text(HARNESS)
        executable = temp / "test_wifi_ap"
        subprocess.run([
            "gcc", "-std=gnu11", "-w", "-DPLATFORM_SV6X66=1",
            "-ffunction-sections", "-fdata-sections", "-include", str(env),
            "-I" + str(stubs), "-I" + str(APP / "src/hal/sv6x66"), str(source), str(harness), "-Wl,--gc-sections",
            "-o", str(executable),
        ], cwd=APP, check=True)
        subprocess.run([str(executable)], cwd=APP, check=True, timeout=10)
    print("SV6X66 open AP DHCP-pool and beacon checks passed")

if __name__ == "__main__":
    main()
