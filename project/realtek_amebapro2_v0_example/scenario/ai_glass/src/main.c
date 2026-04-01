#include "FreeRTOS.h"
#include "task.h"
#include "diag.h"
#include "hal.h"
#include "log_service.h"
#include "video_api.h"
#include <platform_opts.h>
#include <platform_opts_bt.h>
#include "video_boot.h"
#include "mmf2_mediatime_8735b.h"
#include "ai_glass_initialize.h"
#include "ftl_common_api.h"
#include "ai_glass_media.h"

#define UART_LOG_BAUDRATE   3000000;  // set log baud rate as 3000000

#define MP_MODE_DEFAULT 0 // 0: ai glass application, 1: mp mode rf tuning

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

static uint8_t g_mp_mode = MP_MODE_DEFAULT;

/* overwrite log uart baud rate for application. ROM and bootloader will remain 115200
 * set LOGUART_TX_OFF 1 to turn off uart output from application
 */
#include "stdio_port_func.h"
extern hal_uart_adapter_t log_uart;

//#define LOGUART_TX_OFF 1

#if defined(LOGUART_TX_OFF) && (LOGUART_TX_OFF==1)
static void (*user_wputc)(phal_uart_adapter_t puart_adapter, uint8_t tx_data) = (void *)0xffffffff;
static void wputc(phal_uart_adapter_t puart_adapter, uint8_t tx_data)
{
	if ((uint32_t)user_wputc == (uint32_t)hal_uart_wputc) {
		user_wputc(puart_adapter, tx_data);
	}
}

void fUART(void *arg)
{
	int argc = 0;
	char *argv[MAX_ARGC] = {0};
	argc = parse_param(arg, argv);

	if (argc != 2)	{
		return;
	}

	if (strncmp(argv[1], "ON", 2) == 0) {
		user_wputc = hal_uart_wputc;
	} else {
		user_wputc = (void *)0xffffffff;
	}
}

static log_item_t uart_items[] = {
	{"UART", fUART,},
};

void atcmd_uart_init(void)
{
	log_service_add_table(uart_items, sizeof(uart_items) / sizeof(uart_items[0]));
}
#else
static void (*wputc)(phal_uart_adapter_t puart_adapter, uint8_t tx_data) = hal_uart_wputc;
#endif

// AT command handler for MP mode
void fMPMODE(void *arg)
{
    int argc = 0;
    char *argv[MAX_ARGC] = {0};
    
    argc = parse_param(arg, argv);
    
    if (argc < 2) {
        // Query mode: AT+MPMODE?
        printf("+MPMODE: %d (0=AI Glass, 1=MP)\r\n", g_mp_mode);
        return;
    }
    
    // Set mode: AT+MPMODE=0 or AT+MPMODE=1
    uint8_t mode = atoi(argv[1]);
    if (mode > 1) {
        printf("Error: Invalid mode (0=AI Glass, 1=MP)\r\n");
        return;
    }
    
    g_mp_mode = mode;
    
    // Save to flash
    ftl_common_write(FLASH_MP_MODE_BLOCK_BASE, &g_mp_mode, sizeof(g_mp_mode));
    
    printf("OK: MP mode set to %d, Auto reboot\r\n", g_mp_mode);
	sys_reset();
}

// MP mode AT command table
static log_item_t mp_mode_items[] = {
    {"AT+MPMODE", fMPMODE,},
};

// Register MP mode AT command
void atcmd_mp_mode_init(void)
{
    log_service_add_table(mp_mode_items, sizeof(mp_mode_items) / sizeof(mp_mode_items[0]));
}

void log_uart_port_init(int log_uart_tx, int log_uart_rx, uint32_t baud_rate)
{
	baud_rate = UART_LOG_BAUDRATE;  // set log baud rate as 3000000

	hal_status_t ret;
	uint8_t uart_idx;

#if defined(CONFIG_BUILD_NONSECURE) && (CONFIG_BUILD_NONSECURE == 1)
	/* prevent pin confliction */
	uart_idx = hal_uart_pin_to_idx(log_uart_rx, UART_Pin_RX);
	hal_pinmux_unregister(log_uart_rx, (PID_UART0 + uart_idx));
	hal_pinmux_unregister(log_uart_tx, (PID_UART0 + uart_idx));
#endif

	//* Init the UART port hadware
	ret = hal_uart_init(&log_uart, log_uart_tx, log_uart_rx, NULL);
	if (ret == HAL_OK) {
		hal_uart_set_baudrate(&log_uart, baud_rate);
		hal_uart_set_format(&log_uart, 8, UartParityNone, 1);

		// hook the putc function to stdio port for printf
#if defined(CONFIG_BUILD_NONSECURE) && (CONFIG_BUILD_NONSECURE == 1)
		stdio_port_init((void *)&log_uart, (stdio_putc_t)wputc, (stdio_getc_t)&hal_uart_rgetc);
#else
		stdio_port_init_s((void *)&log_uart, (stdio_putc_t)wputc, (stdio_getc_t)&hal_uart_rgetc);
		stdio_port_init_ns((void *)&log_uart, (stdio_putc_t)wputc, (stdio_getc_t)&hal_uart_rgetc);
#endif
	}
}

void setup(void)
{
// #if CONFIG_WLAN
// #if ENABLE_FAST_CONNECT
// 	wifi_fast_connect_enable(1);
// #else
// 	wifi_fast_connect_enable(0);
// #endif
// 	wlan_network();
// #endif

atcmd_mp_mode_init();

ftl_common_read(FLASH_MP_MODE_BLOCK_BASE, &g_mp_mode, sizeof(g_mp_mode));

if (g_mp_mode == 1)  {
    // MP mode: disable fast connect and initialize network
    wifi_fast_connect_enable(0);
    wlan_network();
}

#if defined(LOGUART_TX_OFF) && (LOGUART_TX_OFF==1)
	atcmd_uart_init();
#endif

}

void set_initial_tick_count(void)
{
	printf("[FOR OTA BOOTLOADER CHECK DIFFERENCE] count = %x\r\n",(*((volatile uint32_t *) 0xe0001004))); //這邊應該會等於1000

	// Check DWT_CTRL(0xe0001000) CYCCNTENA(bit 0). If DWT cycle counter is enabled, set tick count initial value based on DWT cycle counter.
	if ((*((volatile uint32_t *) 0xe0001000)) & 1) {
		(*((volatile uint32_t *) 0xe0001000)) &= (~((uint32_t) 1)); // stop DWT cycle counter
		uint32_t dwt_cyccnt = (*((volatile uint32_t *) 0xe0001004));
		uint32_t systick_load = (configCPU_CLOCK_HZ / configTICK_RATE_HZ) - 1UL;
		initial_tick_count = dwt_cyccnt / systick_load;
	}

	// Auto set the media time offset
	video_boot_stream_t *isp_fcs_info;
	video_get_fcs_info(&isp_fcs_info);  //Get the fcs info
	uint32_t media_time_ms = initial_tick_count + isp_fcs_info->fcs_start_time;
	printf("media_time_ms = %d\r\n", media_time_ms);
	mm_set_mediatime_in_ms(media_time_ms);
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
#include "time.h"

void main(void)
{
	/* for debug, protect rodata*/
	//mpu_rodata_protect_init();
	console_init();

	voe_t2ff_prealloc();

	setup();

	/* Execute ai glass example */
	if (g_mp_mode == 1) {
		printf("Entered MP mode\r\n");
	} else {
		ai_glass_init();
	}

	extern void sys_backtrace_enable(void);
	sys_backtrace_enable();

	/* set tick count initial value before start scheduler */
	set_initial_tick_count();
	vTaskStartScheduler();
	while (1);
}