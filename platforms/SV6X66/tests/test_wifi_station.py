#!/usr/bin/env python3
"""Check the actual Wi-Fi HAL against OpenBeken's DHCP/static-IP contract."""
from pathlib import Path
import subprocess
import tempfile
from test_wifi_ap import APP, ENV, HEADERS

HARNESS = r'''#include "hal/hal_wifi.h"
#include <assert.h>
#include <stdio.h>
static int mode = DUT_STA, connect_calls, status;
static u8 use_dhcp;
static u32 address, netmask, route, resolver;
IEEE80211STATUS gwifistatus;
int get_DUT_wifi_mode(void) { return mode; }
int DUT_wifi_start(WIFI_OPMODE next) { mode = next; return 0; }
int set_if_config_2(u8 id,u8 dhcp,u32 ip,u32 mask,u32 gw,u32 dns) {
    assert(id == 0); use_dhcp=dhcp; address=ip; netmask=mask; route=gw; resolver=dns;
    return 0;
}
int get_if_config_2(char *name,u8 *dhcp,u32 *ip,u32 *mask,u32 *gateway,u32 *dns,u8 *mac,int len) {
    (void)name; (void)len; *dhcp=use_dhcp; *ip=address; *mask=netmask; *gateway=route; *dns=resolver;
    memset(mac,0,6); return 0;
}
int wifi_connect_active_5(u8 *ssid,u8 slen,u8 *key,u8 klen,u8 security,u8 channel,u8 *mac,u8 reconnect,u8 rssi,void (*cb)(WIFI_RSP *)) {
    (void)security; (void)channel; (void)mac; (void)reconnect; (void)rssi; (void)cb;
    assert(slen == 8 && memcmp(ssid,"test-net",8)==0);
    assert(klen == 0 || (klen == 8 && memcmp(key,"password",8)==0));
    ++connect_calls; return 0;
}
static void changed(int code) { status=code; }
int main(void) {
    obkStaticIP_t ip = {0};
    HAL_WiFi_SetupStatusCallback(changed);
    /* Shared Main_ConnectToWiFiNow always passes a non-null, initially zero config. */
    HAL_ConnectToWiFi("test-net","password",&ip);
    assert(use_dhcp == 1 && connect_calls == 1 && status == WIFI_STA_CONNECTING);
    assert(address == 0 && netmask == 0 && route == 0 && resolver == 0);
    const unsigned char local[4]={192,168,1,23}, mask[4]={255,255,255,0};
    const unsigned char gw[4]={192,168,1,1}, dns[4]={1,1,1,1};
    memcpy(ip.localIPAddr,local,4); memcpy(ip.netMask,mask,4);
    memcpy(ip.gatewayIPAddr,gw,4); memcpy(ip.dnsServerIpAddr,dns,4);
    HAL_ConnectToWiFi("test-net","",&ip);
    assert(use_dhcp == 0 && connect_calls == 2);
    assert(memcmp(&address,local,4)==0 && memcmp(&netmask,mask,4)==0);
    assert(memcmp(&route,gw,4)==0 && memcmp(&resolver,dns,4)==0);
    return 0;
}
'''

def main():
    with tempfile.TemporaryDirectory(prefix="sv6166f-wifi-station-") as tmp:
        temp=Path(tmp)
        for name in HEADERS:
            p=temp/name
            p.parent.mkdir(parents=True,exist_ok=True)
            p.write_text("/* declarations provided by env.h */\n")
        (temp/'env.h').write_text(ENV)
        (temp/'harness.c').write_text(HARNESS)
        executable=temp/'station'
        subprocess.run(['gcc','-std=gnu11','-Wall','-Wextra','-Werror','-DPLATFORM_SV6X66=1',
                        '-ffunction-sections','-fdata-sections','-include',str(temp/'env.h'),
                        '-I'+str(temp),'-I'+str(APP/'src'),
                        str(APP/'src/hal/sv6x66/hal_wifi_sv6x66.c'),str(temp/'harness.c'),
                        '-Wl,--gc-sections','-o',str(executable)],check=True)
        subprocess.run([str(executable)],check=True,timeout=10)
    print('SV6166F station DHCP/static-IP contract checks passed')

if __name__ == '__main__':
    main()
