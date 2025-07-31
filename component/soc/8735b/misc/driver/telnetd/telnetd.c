#include "FreeRTOS.h"
#include "task.h"
#include "lwipconf.h"
#include "cmsis.h"
#include "diag.h"
#include "log_service.h"

static int telnetd_sock = -1;
static int client_sock = -1;
static int telnetd_run = 0;
static int telnetd_inited = 0;
static int telnetd_atcmd_inited = 0;

#define TELNETD_BUF_SIZE 1024

static char recv_buffer[2][TELNETD_BUF_SIZE];
static int recv_buffer_idx[2] = {0};
static int recv_buffer_slot = 0;

#define TELNET_IAC   255
#define TELNET_WILL  251
#define TELNET_WONT  252
#define TELNET_DO    253
#define TELNET_DONT  254

void at_telnetd_init(void);

static SemaphoreHandle_t wr_mutex = NULL;

unsigned telnetd_read_buffer(unsigned fd, void *buf, unsigned len)
{
	// report current read buffer
	int curr_slot;
	unsigned cnt;

	__disable_irq();
	curr_slot = recv_buffer_slot;
	recv_buffer_slot ^= 1;
	__enable_irq();

	cnt = (unsigned)recv_buffer_idx[curr_slot];
	if (cnt > 0) {
		memcpy(buf, recv_buffer[curr_slot], cnt);
	}
	recv_buffer_idx[curr_slot] = 0;

	return cnt;
}

#define TELNETD_SEND_BUF_SIZE 4096
static char send_buffer[2][TELNETD_SEND_BUF_SIZE];
static int send_buffer_idx[2] = {0};
static int send_buffer_slot = 0;

extern int check_in_critical(void);
unsigned telnetd_write_buffer(unsigned fd, const void *buf, unsigned len)
{

	// write socket
	if (wr_mutex && buf && len > 0) {
		int in_critical = check_in_critical();
		if (in_critical == 0) {
			xSemaphoreTake(wr_mutex, portMAX_DELAY);
		}

		//__disable_irq();
		unsigned cnt = (unsigned)send_buffer_idx[send_buffer_slot];
		char *tmp = send_buffer[send_buffer_slot];
		int copy_size = len > TELNETD_SEND_BUF_SIZE - cnt ? TELNETD_SEND_BUF_SIZE - cnt : len;
		if (copy_size > 0) {
			memcpy(&tmp[cnt], buf, copy_size);
			send_buffer_idx[send_buffer_slot] += copy_size;
		}
		//__enable_irq();
		if (in_critical == 0) {
			xSemaphoreGive(wr_mutex);
		}
	}

	return len;
}

int iac_handling(int sock, uint8_t *buf, int len)
{
	// simple IAC handing, not support enabling feature
	// WILL -> DONT
	// DO -> WONT
	// WONT -> DONT
	// DONT -> WONT

	dbg_printf("IAC %02x %02x %02x\n\r", buf[0], buf[1], buf[2]);
	switch (buf[1]) {
	case TELNET_WILL:
		buf[1] = TELNET_DONT;
		break;
	case TELNET_WONT:
		buf[1] = TELNET_DONT;
		break;
	case TELNET_DO:
		buf[1] = TELNET_WONT;
		break;
	case TELNET_DONT:
		buf[1] = TELNET_WONT;
		break;
	}

	write(sock, buf, 3);
	return 0;
}

uint8_t *iac_data(uint8_t *buf, int act, int function)
{
	buf[0] = 0xFF;
	buf[1] = (uint8_t)act;
	buf[2] = (uint8_t)function;
	return buf;
}

// serve only one connection
void telnetd_handler(void *dummy)
{
	(void)dummy;

	if (telnetd_atcmd_inited == 0) {
		// delay init atcmd.
		telnetd_atcmd_inited = 1;
		wr_mutex = xSemaphoreCreateMutex();
		at_telnetd_init();
	}

	if ((telnetd_sock = socket(AF_INET, SOCK_STREAM, 0)) >= 0) {
		struct sockaddr_in telnetd_addr;
		int enable = 1;
		int ret = 0;

		telnetd_addr.sin_family = AF_INET;
		telnetd_addr.sin_port = htons(23);
		telnetd_addr.sin_addr.s_addr = INADDR_ANY;

		if (setsockopt(telnetd_sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&enable, sizeof(enable)) != 0) {
			dbg_printf("telnetd: SO_REUSEADDR fail\n\r");
		}

		if ((ret = bind(telnetd_sock, (struct sockaddr *) &telnetd_addr, sizeof(telnetd_addr))) != 0) {
			dbg_printf("telnetd: bind fail\n\r");
			goto exit;
		}

		if ((ret = listen(telnetd_sock, 1)) != 0) {
			dbg_printf("telnetd: listen fail\n\r");
			goto exit;
		}
	} else {
		dbg_printf("telnetd: socket open fail\n\r");
		goto exit;
	}

	dbg_printf("telnetd: started\n\r");

	telnetd_run = 1;
	while (telnetd_run) {
		// fd_set size would depend on MEMP_NUM_NETCONN from lwipopts.h, which might be modified by customer hence would make the fd_set size not consistent from customer SDK and HTTP library.
		// To prevent the inconsistency, reserve more dummy space for fd_set here, so that it can handle up to MEMP_NUM_NETCONN 128 from customer SDK.
		union {
			fd_set fds;
			char dummy[16];
		} r;
		struct timeval timeout;

		memset(r.dummy, 0, sizeof(r.dummy));

		timeout.tv_sec = 0;
		timeout.tv_usec = 20000;
		FD_ZERO(&r.fds);
		FD_SET(telnetd_sock, &r.fds);

		if (client_sock > 0) {
			FD_SET(client_sock, &r.fds);
		}

		int max_sock =  client_sock > telnetd_sock ? client_sock : telnetd_sock;

		if (select(max_sock + 1, &r.fds, NULL, NULL, &timeout)) {
			if (FD_ISSET(telnetd_sock, &r.fds)) {
				struct sockaddr_in client_addr;
				unsigned int client_addr_size = sizeof(client_addr);
				if ((client_sock = accept(telnetd_sock, (struct sockaddr *) &client_addr, &client_addr_size)) >= 0) {
					dbg_printf("telnetd: client connected\n\r");
					dbg_printf("telnetd: ask turn on local echo\n\r");
					uint8_t tmp[8];
					write(client_sock, iac_data(tmp, TELNET_WILL, 0x01), 3);	// will echo
					write(client_sock, iac_data(tmp, TELNET_WILL, 0x03), 3);	// will suppress-go-ahead
				}
			} else if (FD_ISSET(client_sock, &r.fds)) {
				int len = read(client_sock, recv_buffer[recv_buffer_slot], TELNETD_BUF_SIZE);
				if (len > 0) {
					unsigned char *buf = (unsigned char *)recv_buffer[recv_buffer_slot];
					if (buf[0] == TELNET_IAC) {
						iac_handling(client_sock, buf, len);
						len = 0;
					}
					recv_buffer_idx[recv_buffer_slot] = len;
				} else {
					close(client_sock);
					client_sock = -1;
					dbg_printf("telnetd: client disconnected\n\r");
				}
			}
		}
		if (client_sock >= 0) {
			int curr_slot;
			unsigned cnt;
			char *buf;

			__disable_irq();
			curr_slot = send_buffer_slot;
			send_buffer_slot ^= 1;
			__enable_irq();

			buf = send_buffer[curr_slot];

			cnt = (unsigned)send_buffer_idx[curr_slot];
			if (client_sock >= 0 && cnt > 0) {
				write(client_sock, buf, cnt);
			}
			send_buffer_idx[curr_slot] = 0;
		}
	}
exit:
	if (telnetd_sock >= 0) {
		close(telnetd_sock);
	}
	if (wr_mutex) {
		vSemaphoreDelete(wr_mutex);
		wr_mutex = NULL;
	}

	telnetd_sock = -1;
	telnetd_inited = 0;
	vTaskDelete(NULL);
}

void telnetd_init(void)
{
	// create task
	if (telnetd_inited == 0) {
		telnetd_inited = 1;
		xTaskCreate(telnetd_handler, "telnetd", 1024, NULL, 1, NULL);
	}
}

void telnetd_deinit(void)
{
	telnetd_run = 0;
}

void fRCON(void *arg)
{
	extern void console_stdio_init(void *read_cb, void *write_cb);
	extern void remote_stdio_init(void *read_cb, void *write_cb);
	extern unsigned telnetd_read_buffer(unsigned fd, void *buf, unsigned len);
	extern unsigned telnetd_write_buffer(unsigned fd, const void *buf, unsigned len);
	extern unsigned uart_read_buffer(unsigned fd, void *buf, unsigned len);
	extern unsigned uart_write_buffer(unsigned fd, const void *buf, unsigned len);
	extern void telnetd_init(void);
	extern void telnetd_deinit(void);

	int argc = 0;
	char *argv[MAX_ARGC] = {0};
	argc = parse_param(arg, argv);
	if (argc == 2) {
		int ena = atoi(argv[1]);
		if (ena == 1) {
			telnetd_init();
			remote_stdio_init(telnetd_read_buffer, telnetd_write_buffer);
			console_stdio_init(uart_read_buffer, uart_write_buffer);
		} else if (ena == 2) {
			telnetd_init();
			remote_stdio_init(NULL, NULL);
			console_stdio_init(telnetd_read_buffer, telnetd_write_buffer);
		} else {
			telnetd_deinit();
			remote_stdio_init(NULL, NULL);
			console_stdio_init(uart_read_buffer, uart_write_buffer);
		}
	}
}

log_item_t at_telnetd_items[] = {
	{"rcon", fRCON, {NULL, NULL}},
};

void at_telnetd_init(void)
{
	log_service_add_table(at_telnetd_items, sizeof(at_telnetd_items) / sizeof(at_telnetd_items[0]));
}

#ifndef INIT_TELNETD
#define INIT_TELNETD 0
#endif

#if defined(INIT_TELNETD) && (INIT_TELNETD==1)
__attribute__((constructor))
void telnetd_io_init(void)
{
	extern void remote_stdio_init(void *read_cb, void *write_cb);

	telnetd_init();
	remote_stdio_init(telnetd_read_buffer, telnetd_write_buffer);
}

#endif