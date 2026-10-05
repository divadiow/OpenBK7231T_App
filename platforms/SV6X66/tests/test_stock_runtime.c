#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
static volatile uint32_t control;
static unsigned reads;
volatile uint32_t *StockTestRegister(uint32_t addr) {
    assert(addr == 0xC000100C); reads++; return &control;
}
void __wrap__soc_clk_init(void);
void __wrap__soc_io_init(void);
void __wrap_xip_init(void);
void __wrap_xip_leave(void);
void __wrap_xip_enter(void);
int main(int argc, char **argv) {
    assert(argc == 2);
    unsigned mode = (unsigned)atoi(argv[1]); assert(mode <= 6 && !(mode & 1));
    control = 0x5010 | mode;
    __wrap__soc_clk_init(); __wrap__soc_io_init(); assert(reads == 0 && control == (0x5010 | mode));
    __wrap_xip_init(); assert(control == (0x5010 | mode));
    for (int i=0; i<3; i++) {
        __wrap_xip_leave(); assert(control == 0x5010);
        __wrap_xip_init(); // Must not recapture the temporarily disabled mode.
        control |= 0x20000; // Unrelated controller bits changed by an operation.
        __wrap_xip_enter(); assert(control == (0x25010 | mode));
        control &= ~0x20000;
    }
    puts("Stock XIP mode captured once and restored; timing/unrelated bits preserved");
    return 0;
}
