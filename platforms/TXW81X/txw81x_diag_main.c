#if defined(TXW81X_DIAG_MINIMAL)

#include "sys_config.h"
#include "typesdef.h"
#include "devid.h"
#include "dev.h"
#include "hal/gpio.h"
#include "hal/uart.h"
#include "osal/sleep.h"

extern void system_goto_boot(void);

#define MODE_REG (*(volatile uint32 *)0x400200C4u)

static struct uart_device *g_uart;
static volatile uint32 g_nonce;
static volatile uint32 g_armed;

static void txc(uint8 c)
{
    uart_putc(g_uart, (int8)c);
}

static void txs(const char *s)
{
    while (*s) {
        txc((uint8)*s++);
    }
}

static void txnl(void)
{
    txc('\r');
    txc('\n');
}

static void txhex(uint32 v)
{
    int i;
    for (i = 7; i >= 0; --i) {
        uint32 n = (v >> (i * 4)) & 0xFu;
        txc((uint8)(n < 10u ? ('0' + n) : ('A' + n - 10u)));
    }
}

static int hexval(int c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static uint32 parsehex(const char *p, int *ok)
{
    uint32 v = 0;
    int n = 0;
    int h;
    while ((h = hexval((uint8)*p)) >= 0) {
        v = (v << 4) | (uint32)h;
        ++p;
        ++n;
    }
    *ok = n > 0;
    return v;
}

static int streq(const char *a, const char *b)
{
    while (*a && *b && *a == *b) {
        ++a;
        ++b;
    }
    return *a == 0 && *b == 0;
}

static void snapshot(void)
{
    uint32 mode = MODE_REG;
    txs("TXWDIAG SNAP mode=0x"); txhex(mode);
    txs(" cp="); txc((uint8)('0' + ((mode >> 0) & 1u)));
    txs(" ft="); txc((uint8)('0' + ((mode >> 1) & 1u)));
    txs(" chip=0x"); txhex((uint32)sysctrl_get_chip_id());
    txs(" pack=0x"); txhex((uint32)get_chip_pack());
    txs(" bios=0x"); txhex((uint32)get_bios_id());
    txs(" module=0x"); txhex((uint32)sysctrl_efuse_get_module_type());
    txnl();
}

static void process_line(char *s)
{
    int ok;
    uint32 n;

    if (streq(s, "AT+DIAG=PING")) {
        txs("TXWDIAG PONG v=0.3 target=TXW817-810");
        txnl();
        return;
    }

    if (streq(s, "AT+DIAG=SNAPSHOT")) {
        snapshot();
        return;
    }

    if (s[0]=='A' && s[1]=='T' && s[2]=='+' && s[3]=='D' && s[4]=='I' &&
        s[5]=='A' && s[6]=='G' && s[7]=='=' && s[8]=='A' && s[9]=='R' &&
        s[10]=='M' && s[11]==',') {
        n = parsehex(s + 12, &ok);
        if (!ok) {
            txs("TXWDIAG ERROR BAD_NONCE"); txnl();
            return;
        }
        g_nonce = n;
        g_armed = 1;
        snapshot();
        txs("TXWDIAG ARMED 0x"); txhex(n); txnl();
        return;
    }

    if (s[0]=='A' && s[1]=='T' && s[2]=='+' && s[3]=='D' && s[4]=='I' &&
        s[5]=='A' && s[6]=='G' && s[7]=='=' && s[8]=='E' && s[9]=='X' &&
        s[10]=='E' && s[11]=='C' && s[12]==',') {
        n = parsehex(s + 13, &ok);
        if (!ok || !g_armed || n != g_nonce) {
            txs("TXWDIAG ERROR NOT_ARMED"); txnl();
            return;
        }
        g_armed = 0;
        snapshot();
        txs("TXWDIAG ROMGO 0x"); txhex(n); txnl();
        os_sleep_ms(100);
        system_goto_boot();
        for (;;) {}
    }

    txs("TXWDIAG ERROR UNKNOWN");
    txnl();
}

static void move_uart0_to_pa8_pa9(void)
{
    /* device_init() has already attached/opened UART0 on the normal OpenTXW81X
       console pins. Re-route the already-live UART to the ROM UART pins, then
       change only its baud rate. This deliberately avoids reopening UART0,
       because uart_open() would reapply project_config.h's PC7/PC6 pin map. */
    gpio_iomap_output(PA_8, GPIO_IOMAP_OUT_UART0_TX);
    gpio_iomap_input(PA_9, GPIO_IOMAP_IN_UART0_IN);
    gpio_set_mode(PA_9, GPIO_PULL_UP, GPIO_PULL_LEVEL_100K);
    uart_ioctl(g_uart, UART_IOCTL_CMD_SET_BAUDRATE, 115200, 0);
}

int main(void)
{
    char line[96];
    uint32 used = 0;

    g_uart = (struct uart_device *)dev_get(HG_UART0_DEVID);
    if (!g_uart) {
        for (;;) {}
    }

    move_uart0_to_pa8_pa9();
    os_sleep_ms(20);

    txs("TXWDIAG READY v=0.3 target=TXW817-810 PA8/PA9 115200");
    txnl();
    snapshot();

    for (;;) {
        int c = (int)uart_getc(g_uart);
        if (c == '\r' || c == '\n') {
            if (used) {
                line[used] = 0;
                process_line(line);
                used = 0;
            }
        } else if (used + 1u < sizeof(line)) {
            line[used++] = (char)c;
        } else {
            used = 0;
        }
    }
}

#endif
