#include "FreeRTOS.h"
#include "task.h"
#include "osdep_service.h"
#include "lwip/sockets.h"
#include "ssh.h"
#include "sshcrypto.h"
#include "channel.h"
#include "packet.h"
#include "global.h"

#define NONEAUTH 0
#define DEBUG 0

int tinyssh_server_fd = -1, tinyssh_client_fd = -1;
uint8_t tinyssh_stdio_used = 0;
uint8_t tinyssh_running = 0;
uint8_t tinyssh_stopping = 0;
TaskHandle_t tinyssh_task_handle = NULL;

unsigned char tinyssh_sign_publickey[crypto_sign_ed25519_PUBLICKEYBYTES] = {
	0x0a, 0xf2, 0x58, 0xc0, 0x5e, 0xeb, 0x5e, 0xb1, 0x5a, 0xbf, 0x56, 0xa2, 0x7e, 0x58, 0xdd, 0x11, 0x9b, 0x20, 0x6f, 0xd8, 0x1b, 0x25, 0xcb, 0x5a, 0xbd, 0x9c, 0x0e, 0x61, 0x57, 0x0c, 0xf6, 0xba
};
unsigned char tinyssh_sign_secretkey[crypto_sign_ed25519_SECRETKEYBYTES] = {
	0xa7, 0x32, 0xa3, 0x11, 0xb2, 0x13, 0xa3, 0x11, 0x13, 0x19, 0xa3, 0x11, 0x99, 0x49, 0xa3, 0x11, 0xc9, 0x0c, 0xa3, 0x11, 0x8c, 0x24, 0xa3, 0x11, 0x24, 0x06, 0xa3, 0x11, 0x06, 0x52, 0xa3, 0x11,
	0x0a, 0xf2, 0x58, 0xc0, 0x5e, 0xeb, 0x5e, 0xb1, 0x5a, 0xbf, 0x56, 0xa2, 0x7e, 0x58, 0xdd, 0x11, 0x9b, 0x20, 0x6f, 0xd8, 0x1b, 0x25, 0xcb, 0x5a, 0xbd, 0x9c, 0x0e, 0x61, 0x57, 0x0c, 0xf6, 0xba
};

const char *tinyssh_client_account = "test";
const char *tinyssh_client_keyname = "ssh-ed25519";
const char *tinyssh_client_publickey = "AAAAC3NzaC1lZDI1NTE5AAAAIAVV3UPxYH9EMnq3t9It18+9FU8g9aBGiMxuSxKV99zr";

unsigned char global_bspace3[GLOBAL_BSIZE];
static struct buf b3 = {global_bspace3, 0, sizeof global_bspace3};

_mutex tinyssh_channel_mutex = NULL;
_mutex tinyssh_packet_mutex = NULL;

extern int main_tinysshd(int argc, char **argv, const char *binaryname);

unsigned tinyssh_read_buffer(unsigned fd, void *buf, unsigned len)
{
	unsigned cnt = 0, len_copy = 0;

	if (tinyssh_stdio_used) {
		cnt = (unsigned) channel.len0;
		if (cnt > 0) {
			rtw_mutex_get(&tinyssh_channel_mutex);
			cnt = (unsigned) channel.len0;
			len_copy = (cnt > len) ? len : cnt;
			memcpy(buf, channel.buf0, len_copy);
			if ((cnt - len_copy) > 0) {
				memcpy(channel.buf0, channel.buf0 + len_copy, cnt - len_copy);
			}
			channel.len0 -= len_copy;
			channel.localwindow += len_copy;
			rtw_mutex_put(&tinyssh_channel_mutex);
		}
	}

	return cnt;
}

unsigned tinyssh_write_buffer(unsigned fd, const void *buf, unsigned len)
{
	if (tinyssh_stdio_used) {
		buf_purge(&b3);
		buf_putnum8(&b3, SSH_MSG_CHANNEL_DATA);
		buf_putnum32(&b3, channel.id);
		buf_putnum32(&b3, len);
		buf_put(&b3, (uint8_t *) buf, len);
		rtw_mutex_get(&tinyssh_packet_mutex);
		packet_put(&b3);
		rtw_mutex_put(&tinyssh_packet_mutex);
		buf_purge(&b3);
	}

	return len;
}

int tinyssh_check_recv(void)
{
	// fd_set size would depend on MEMP_NUM_NETCONN from lwipopts.h, which might be modified by customer hence would make the fd_set size not consistent from customer SDK and HTTP library.
	// To prevent the inconsistency, reserve more dummy space for fd_set here, so that it can handle up to MEMP_NUM_NETCONN 128 from customer SDK.
	union {
		fd_set fds;
		char dummy[16];
	} r;
	struct timeval timeout;

	memset(r.dummy, 0, sizeof(r.dummy));

	timeout.tv_sec = 0;
	timeout.tv_usec = 100000;
	FD_ZERO(&r.fds);
	FD_SET(tinyssh_client_fd, &r.fds);

	return select(tinyssh_client_fd + 1, &r.fds, NULL, NULL, &timeout);
}

void tinyssh_task(void *param)
{
#if NONEAUTH
#if DEBUG
	char *args[] = {(char *) "tinysshnoneauthd", (char *) "-v", (char *) "-v", (char *) "./keydir"};
	main_tinysshd(4, args, args[0]);
#else
	char *args[] = {(char *) "tinysshnoneauthd", (char *) "./keydir"};
	main_tinysshd(2, args, args[0]);
#endif
#else
#if DEBUG
	char *args[] = {(char *) "tinysshd", (char *) "-v", (char *) "-v", (char *) "./keydir"};
	main_tinysshd(4, args, args[0]);
#else
	char *args[] = {(char *) "tinysshd", (char *) "./keydir"};
	main_tinysshd(2, args, args[0]);
#endif
#endif

	tinyssh_task_handle = NULL;
	vTaskDelete(NULL);
}

void tinysshd(void *param)
{
	/* To avoid gcc warnings */
	(void) param;

	int ret = 0;

	tinyssh_running = 1;
	tinyssh_stopping = 0;
	rtw_mutex_init(&tinyssh_channel_mutex);
	rtw_mutex_init(&tinyssh_packet_mutex);
	extern void remote_stdio_init(void *read_cb, void *write_cb);
	remote_stdio_init(tinyssh_read_buffer, tinyssh_write_buffer);

	if ((tinyssh_server_fd = socket(AF_INET, SOCK_STREAM, 0)) >= 0) {
		struct sockaddr_in server_addr;
		int enable = 1;

		server_addr.sin_family = AF_INET;
		server_addr.sin_port = htons(22);
		server_addr.sin_addr.s_addr = INADDR_ANY;

		if (setsockopt(tinyssh_server_fd, SOL_SOCKET, SO_REUSEADDR, (const char *)&enable, sizeof(enable)) != 0) {
			printf("ERROR: SO_REUSEADDR\n\r");
		}

		if ((ret = bind(tinyssh_server_fd, (struct sockaddr *) &server_addr, sizeof(server_addr))) != 0) {
			printf("ERROR: bind\n\r");
			goto exit;
		}

		if ((ret = listen(tinyssh_server_fd, 3)) != 0) {
			printf("ERROR: listen\n\r");
			goto exit;
		}

		printf("\n\rtinysshd started\n\r");

		while (1) {
			// fd_set size would depend on MEMP_NUM_NETCONN from lwipopts.h, which might be modified by customer hence would make the fd_set size not consistent from customer SDK and HTTP library.
			// To prevent the inconsistency, reserve more dummy space for fd_set here, so that it can handle up to MEMP_NUM_NETCONN 128 from customer SDK.
			union {
				fd_set fds;
				char dummy[16];
			} r;
			struct timeval timeout;

			memset(r.dummy, 0, sizeof(r.dummy));

			timeout.tv_sec = 1;
			timeout.tv_usec = 0;
			FD_ZERO(&r.fds);
			FD_SET(tinyssh_server_fd, &r.fds);

			if (select(tinyssh_server_fd + 1, &r.fds, NULL, NULL, &timeout) == 1) {
				struct sockaddr_in client_addr;
				unsigned int client_addr_size = sizeof(client_addr);

				if ((tinyssh_client_fd = accept(tinyssh_server_fd, (struct sockaddr *)&client_addr, &client_addr_size)) >= 0) {

					if (xTaskCreate(tinyssh_task, ((const char *) "tinyssh_task"), 2048, NULL, tskIDLE_PRIORITY + 1, &tinyssh_task_handle) != pdPASS) {
						printf("\n\r%s xTaskCreate(tinyssh_task) failed\n", __FUNCTION__);
					}

					while (1) {
						if (tinyssh_task_handle == NULL) {
#if DEBUG
							printf("\n\r%s close tinyssh_client_fd(%d)\n", __func__, tinyssh_client_fd);
#endif
							close(tinyssh_client_fd);
							tinyssh_client_fd = -1;
							break;
						}
						vTaskDelay(1000);
					}
				}
			}

			if (tinyssh_stopping) {
				break;
			}
		}
	} else {
		printf("ERROR: socket\n\r");
		goto exit;
	}

exit:
	if (tinyssh_server_fd != -1) {
		close(tinyssh_server_fd);
		tinyssh_server_fd = -1;
	}

	remote_stdio_init(NULL, NULL);
	rtw_mutex_free(&tinyssh_channel_mutex);
	rtw_mutex_free(&tinyssh_packet_mutex);
	tinyssh_running = 0;
	tinyssh_stopping = 0;
	printf("\n\rtinysshd stopped\n\r");
	vTaskDelete(NULL);
}

void tinysshd_start(void)
{
	if (tinyssh_running == 1) {
		printf("\n\rtinysshd is already running\n\r");
		return;
	}

#if 0
	char *args[] = {(char *) "tinysshd-makekey", (char *) "./keydir"};
	extern int main_tinysshd_makekey(int argc, char **argv);
	main_tinysshd_makekey(2, args);
#endif

	if (xTaskCreate(tinysshd, ((const char *) "tinysshd"), 2048, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(tinysshd) failed\n", __FUNCTION__);
	}
}

void tinysshd_stop(void)
{
	if (tinyssh_running == 1) {
		tinyssh_stopping = 1;

		for (int i = 0; i < 50; i ++) {
			if (tinyssh_running == 0) {
				return;
			}
			vTaskDelay(100);
		}
		printf("\n\rERROR: %s timeout\n\r", tinysshd_stop);
	}
}
