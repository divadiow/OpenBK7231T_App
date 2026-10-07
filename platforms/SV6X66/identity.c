#include <stdint.h>
#include <string.h>
#include "identity.h"
#include "bsp/soc/efuseapi/efuse_api.h"

/* This SDK archive exports _increase; efuse_api.h incorrectly declares _inc.
 * The SDK radio uses _increase for its second interface (last octet XOR 1). */
extern int efuse_read_mac_increase(uint8_t *mac_addr);

static uint8_t interface_mac[2][6];
static int identity_ready;

static int valid_mac(const uint8_t mac[6])
{
    static const uint8_t zero[6];
    /* Unicast excludes broadcast as well as multicast; zero is not identity. */
    return !(mac[0] & 1) && memcmp(mac, zero, sizeof(zero)) != 0;
}

int SV6X66_IdentityInit(void)
{
    uint8_t pair[2][6] = {{0}};
    if (identity_ready) return 1;
    if (efuse_read_mac(pair[0]) != 0 || efuse_read_mac_increase(pair[1]) != 0)
        return 0;
    if (!valid_mac(pair[0]) || !valid_mac(pair[1]) ||
        memcmp(pair[0], pair[1], 5) != 0 ||
        pair[1][5] != (uint8_t)(pair[0][5] ^ 1))
        return 0;
    memcpy(interface_mac, pair, sizeof(pair));
    identity_ready = 1;
    return 1;
}

/* WIFI_INIT's radio reads these getters after IdentityInit succeeds. Wrapping
 * both getters avoids the application trailer's fixed demonstration addresses
 * without modifying vendor configuration, stock flash layout or efuses. */
void __wrap_wifi_cfg_get_addr1(const void *handle, char addr[6])
{
    (void)handle;
    memcpy(addr, interface_mac[0], 6);
}

void __wrap_wifi_cfg_get_addr2(const void *handle, char addr[6])
{
    (void)handle;
    memcpy(addr, interface_mac[1], 6);
}
