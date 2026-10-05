#include <stdint.h>
volatile uint32_t *StockTestRegister(uint32_t addr);
#define SV6X66_REG32(addr) (*StockTestRegister(addr))
