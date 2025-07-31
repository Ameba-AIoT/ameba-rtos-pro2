#include "power_mode_api.h"
#include <string.h>

__attribute__((section(".retention.data"))) uint16_t retention_magic __attribute__((aligned(32)));
__attribute__((section(".retention.data"))) uint32_t retention_count __attribute__((aligned(32)));

int main(void)
{
	hal_xtal_divider_enable(1);
	hal_32k_s1_sel(2);
	dbg_printf("\r\n   PM Retention DEMO   \r\n");

	if (retention_magic == 0x1234) {
		retention_count ++;
		dcache_clean_invalidate_by_addr((uint32_t *) &retention_count, sizeof(retention_count));
	} else {
		retention_magic = 0x1234;
		dcache_clean_invalidate_by_addr((uint32_t *) &retention_magic, sizeof(retention_magic));
		retention_count = 0;
		dcache_clean_invalidate_by_addr((uint32_t *) &retention_count, sizeof(retention_count));
	}

	dbg_printf("\r\n retention_count=%u \r\n", retention_count);

	// SLP_AON_TIMER to wakeup after 5s
	// SLP_GTIMER for SRAM retention
	HAL_WRITE32(0x40009000, 0x18, 0x1 | HAL_READ32(0x40009000, 0x18)); //SWR 1.35V
	Standby(SLP_AON_TIMER | SLP_GTIMER, 5000000 /* 5s */, 0 /* CLOCK */, 1 /* SRAM retention */);

	while (1);
}
