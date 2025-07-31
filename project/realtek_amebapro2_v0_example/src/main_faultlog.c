#include "FreeRTOS.h"
#include "task.h"
#include "diag.h"
#include "hal.h"
#include "log_service.h"
#include "video_api.h"
#include <platform_opts.h>
#include <platform_opts_bt.h>

#if CONFIG_WLAN
#include <wifi_fast_connect.h>
extern void wlan_network(void);
#endif

extern void console_init(void);
extern void mpu_rodata_protect_init(void);

// tick count initial value used when start scheduler
uint32_t initial_tick_count = 0;

#ifdef _PICOLIBC__
int errno;
#endif

#if defined(CONFIG_FTL_ENABLED)
#include <ftl_int.h>

const u8 ftl_phy_page_num = 3;	/* The number of physical map pages, default is 3: BT_FTL_BKUP_ADDR, BT_FTL_PHY_ADDR1, BT_FTL_PHY_ADDR0 */
const u32 ftl_phy_page_start_addr = BT_FTL_BKUP_ADDR;

void app_ftl_init(void)
{
	ftl_init(ftl_phy_page_start_addr, ftl_phy_page_num);
}
#endif

/* entry for the example*/
__weak void app_example(void) {}
void run_app_example(void)
{
#if !defined(CONFIG_UNITEST) || (CONFIG_UNITEST==0)
	/* Execute application example */
	app_example();
#endif
}

void setup(void)
{
#if CONFIG_WLAN
#if ENABLE_FAST_CONNECT
	wifi_fast_connect_enable(1);
#else
	wifi_fast_connect_enable(0);
#endif
	wlan_network();
#endif

#if defined(CONFIG_FTL_ENABLED)
	app_ftl_init();
#endif
}

void set_initial_tick_count(void)
{
	// Check DWT_CTRL(0xe0001000) CYCCNTENA(bit 0). If DWT cycle counter is enabled, set tick count initial value based on DWT cycle counter.
	if ((*((volatile uint32_t *) 0xe0001000)) & 1) {
		(*((volatile uint32_t *) 0xe0001000)) &= (~((uint32_t) 1)); // stop DWT cycle counter
		uint32_t dwt_cyccnt = (*((volatile uint32_t *) 0xe0001004));
		uint32_t systick_load = (configCPU_CLOCK_HZ / configTICK_RATE_HZ) - 1UL;
		initial_tick_count = dwt_cyccnt / systick_load;
	}
}

/**
  * Fault and backtrace log saving and loading example
  */

extern void fault_message_log_init(void(*fault_log)(char *msg, int len), void(*bt_log)(char *msg, int len));
extern void sys_backtrace_enable(void);

// NOTE: this offset is safe on pure SDK, and may conflict with user's layout
//       please check layout to solve confliction before using
#define FAULT_LOG1				(USER_DATA_BASE + 0x64000) //Store fault log(max size: 8K, 0xF64000~0xF66000)
#define FAULT_LOG2				(USER_DATA_BASE + 0x66000) //Store fault log(max size: 8K, 0xF66000~0xF68000)

#include <ftl_common_api.h>
void fault_log(char *msg, int len)
{
	// NOTE: NO OS task related code, no FileSystem, only dbg_printf no printf
	//dbg_printf("%s", msg);
	//dbg_printf("NEW VERSION001\n\r", msg);

	// NOTE: customized string, do not insert NULL or \0 into this area, max 32byte
	//       string api may add NULL or \0 to buffer, please make sure there is no NULL or \0 be inserted
	memcpy(&msg[11], "This is fault", strlen("This is fault"));

	ftl_common_write(FAULT_LOG1, (unsigned char *)msg, len);
}

void bt_log(char *msg, int len)
{
	// NOTE: NO OS task related code, no FileSystem, only dbg_printf no printf
	//dbg_printf("%s", msg);
	//dbg_printf("NEW VERSION002\n\r", msg);

	// NOTE: customized string, do not insert NULL or \0 into this area, max 32byte
	//       string api may add NULL or \0 to buffer, please make sure there is no NULL or \0 be inserted
	memcpy(&msg[11], "This is backtrace", strlen("This is backtrace"));

	ftl_common_write(FAULT_LOG2, (unsigned char *)msg, len);
}

void read_last_fault_log(void)
{
	char *log = malloc(8192);
	if (!log)	{
		return;
	}
	memset(log, 0xff, 8192);
	ftl_common_read(FAULT_LOG1, (unsigned char *)log, 8192);
	if (memcmp(log, "LOG0", 4) == 0) {
		dbg_printf(">>>>>>> Dump fault log <<<<<<<<\n\r");
		dbg_printf("%s", log);
	}
	ftl_common_read(FAULT_LOG2, (unsigned char *)log, 8192);
	if (memcmp(log, "LOG1", 4) == 0) {
		dbg_printf(">>>>>>> Dump Backtrace  log <<<<<<<<\n\r");
		dbg_printf("%s", log);
	}

	memset(log, 0xff, 8192);
	ftl_common_write(FAULT_LOG1, (unsigned char *)log, 8192);
	ftl_common_write(FAULT_LOG2, (unsigned char *)log, 8192);

	free(log);
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

void main(void)
{
	// read last fault log, setup fault and backtrace saving, enable backtrace
	read_last_fault_log();
	fault_message_log_init(fault_log, bt_log);
	sys_backtrace_enable();

	mpu_rodata_protect_init();
	console_init();

	voe_t2ff_prealloc();

	setup();

	/* Execute application example */
	run_app_example();

	/* set tick count initial value before start scheduler */
	set_initial_tick_count();
	vTaskStartScheduler();
	while (1);
}
