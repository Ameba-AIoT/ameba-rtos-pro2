/*
20140120
Jan Mojzis
Public domain.
*/

#include "osdep_service.h"
#include "lwip/sockets.h"

//#include <unistd.h>
#include "writeall.h"
#include "e.h"
#include "byte.h"
#include "purge.h"
#include "packet.h"

extern int tinyssh_client_fd;
extern _mutex tinyssh_packet_mutex;

int packet_sendisready(void)
{

	return (packet.sendbuf.len > 0);
}


int packet_send(void)
{

	struct buf *sendbuf = &packet.sendbuf;
	long long w;

	if (sendbuf->len <= 0) {
		return 1;
	}
	rtw_mutex_get(&tinyssh_packet_mutex);
	w = write(tinyssh_client_fd, sendbuf->buf, sendbuf->len);
	rtw_mutex_put(&tinyssh_packet_mutex);
	if (w == -1) {
		if (errno == EINTR) {
			return 1;
		}
		if (errno == EAGAIN) {
			return 1;
		}
		if (errno == EWOULDBLOCK) {
			return 1;
		}
		return 0;
	}
	rtw_mutex_get(&tinyssh_packet_mutex);
	byte_copy(sendbuf->buf, sendbuf->len - w, sendbuf->buf + w);
	sendbuf->len -= w;
	purge(sendbuf->buf + sendbuf->len, w);
	rtw_mutex_put(&tinyssh_packet_mutex);
	return 1;
}

int packet_sendall(void)
{

	if (writeall(tinyssh_client_fd, packet.sendbuf.buf, packet.sendbuf.len) == -1) {
		return 0;
	}
	purge(packet.sendbuf.buf, packet.sendbuf.len);
	packet.sendbuf.len = 0;
	return 1;
}
