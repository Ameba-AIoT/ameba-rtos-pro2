/*
20140120
Jan Mojzis
Public domain.
*/

#include "lwip/sockets.h"

//#include <unistd.h>
#include "e.h"
#include "buf.h"
#include "purge.h"
#include "packet.h"

extern int tinyssh_client_fd;

int packet_recvisready(void)
{

	return buf_ready(&packet.recvbuf, PACKET_FULLLIMIT);
}

int packet_recv(void)
{

	long long r;
	struct buf *b = &packet.recvbuf;

	if (b->len < PACKET_ZEROBYTES) {
		b->len = PACKET_ZEROBYTES;
		purge(b->buf, PACKET_ZEROBYTES);
	}
	if (!packet_recvisready()) {
		return 1;
	}

	r = read(tinyssh_client_fd, b->buf + b->len, PACKET_FULLLIMIT);
	if (r == 0) {
		errno = ECONNRESET;
		return 0;
	}
	if (r == -1) {
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
	b->len += r;
	return 1;
}
