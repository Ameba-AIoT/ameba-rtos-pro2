#include <platform_stdlib.h>
#include <platform_opts.h>
#include "log_service.h"
#include "atcmd_mp.h"

#if defined(CONFIG_ATCMD_MP) && CONFIG_ATCMD_MP
extern void fATM2(void *arg);	// MP ext2 AT command
//-------- AT MP commands ---------------------------------------------------------------
void fATMt(void *arg)
{
	int argc = 0;
	char *argv[MAX_ARGC] = {0};

	(void)argc;

	AT_PRINTK("[ATM#]: _AT_MP_TEST_");
	argc = parse_param(arg, argv);
}

void fATMx(void *arg)
{
	(void)arg;
	AT_PRINTK("[ATM?]: _AT_MP_HELP_");
}

log_item_t at_mp_items[] = {
	{"ATM#", fATMt, {NULL, NULL}},	// test command
	{"ATM?", fATMx, {NULL, NULL}},	// Help
	{"ATM2", fATM2, {NULL, NULL}},	// MP ext2 AT command
};

void at_mp_init(void)
{
	log_service_add_table(at_mp_items, sizeof(at_mp_items) / sizeof(at_mp_items[0]));
}

#if SUPPORT_LOG_SERVICE
log_module_init(at_mp_init);
#endif

#endif // #if CONFIG_ATCMD_MP