#include <stdint.h>
#if defined(SV6X66_STOCK_CKW04)
#ifndef SV6X66_REG32
#define SV6X66_REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))
#endif
#define RAM_BOOT __attribute__((section(".fast_boot_code")))
#define FLASH_CONTROL 0xC000100Cu
#define MODE_MASK 0x6u
// Factory flash routines at 30000834/3000085C wait for these busy bits.
#define BUSY_MASK 0x9u
static uint32_t inherited_mode;
static int captured;
// Called before C data initialization. The stock loader already set 25/80 MHz.
// Do not treat its code at 30000004/08 as the SDK's clock header.
void RAM_BOOT __wrap__soc_clk_init(void) {}
// The SDK board selects UART0_II; the stock loader uses UART0_I.
void RAM_BOOT __wrap__soc_io_init(void) {}
void RAM_BOOT __wrap_xip_init(void)
{
    if (!captured) {
        inherited_mode = SV6X66_REG32(FLASH_CONTROL) & MODE_MASK;
        captured = 1;
    }
    // Preserve the loader's read-command/timing configuration at C000101C.
}
void RAM_BOOT __wrap_xip_leave(void)
{
    __wrap_xip_init();
    while (SV6X66_REG32(FLASH_CONTROL) & BUSY_MASK) {}
    SV6X66_REG32(FLASH_CONTROL) &= ~MODE_MASK;
}
void RAM_BOOT __wrap_xip_enter(void)
{
    __wrap_xip_init();
    while (SV6X66_REG32(FLASH_CONTROL) & BUSY_MASK) {}
    SV6X66_REG32(FLASH_CONTROL) =
        (SV6X66_REG32(FLASH_CONTROL) & ~MODE_MASK) | inherited_mode;
    while (SV6X66_REG32(FLASH_CONTROL) & BUSY_MASK) {}
}
#endif
