#include "sv6x66_loader.h"

void APP_Init(void)
{
	SV6X66_Loader();
}

int lowpower_sleep_gpio_hook(void) { return 0; }
int lowpower_dormant_gpio_hook(void) { return 0; }
void lowpower_pre_sleep_hook(void) {}
void lowpower_post_sleep_hook(void) {}
void vAssertCalled(const char *func, int line)
{
	(void)func;
	(void)line;
	for (;;) {}
}
