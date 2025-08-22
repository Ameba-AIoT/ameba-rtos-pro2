/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2001-2022 Georges Menie (www.menie.org)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* this code needs standard functions memcpy() and memset()
   and input/output functions _inbyte() and _outbyte().

   the prototypes of the input/output functions are:
     int _inbyte(unsigned short timeout); // msec timeout
     void _outbyte(int c);

 */

#include <string.h>
#include "crc16.h"
#include "xmodem.h"


#define SOH  0x01
#define STX  0x02
#define EOT  0x04
#define ACK  0x06
#define BS	 0x08
#define NAK  0x15
#define CAN  0x18
#define CTRLZ 0x1A

#define DLY_1S 1000
#define DLY_100MS 100
#define MAXRETRANS 25

static int dft_inbyte(unsigned short timeout)
{
	return 0;
}

static void dft_outbyte(int timeout)
{
	return;
}

static int (*_inbyte)(unsigned short timeout) = dft_inbyte;
static void (*_outbyte)(int c) = dft_outbyte;

void xmodemSetInterface(int (*in_cb)(unsigned short), void (*out_cb)(int))
{
	if (in_cb) {
		_inbyte = in_cb;
	}
	if (out_cb) {
		_outbyte = out_cb;
	}
}

static int check(int crc, const unsigned char *buf, int sz)
{
	if (crc) {
		unsigned short crc = crc16_ccitt(buf, sz);
		unsigned short tcrc = (buf[sz] << 8) + buf[sz + 1];
		if (crc == tcrc) {
			return 1;
		}
	} else {
		int i;
		unsigned char cks = 0;
		for (i = 0; i < sz; ++i) {
			cks += buf[i];
		}
		if (cks == buf[sz]) {
			return 1;
		}
	}

	return 0;
}

static void flushinput(void)
{
	//while (_inbyte(((DLY_1S)*3)>>1) >= 0)
	//	;
	while (_inbyte(0) >= 0);
}

int dbg_pkg_no = 2;
// mode : 0, start, 1, middle, 2, end
static int recv_packetno = 0;
static int recv_bufsz = 0;
static unsigned char recv_crc = 0;
int xmodemReceive(unsigned char *dest, int destsz, int mode, int *eot)
{
	unsigned char xbuff[1030]; /* 1024 for XModem 1k + 3 head chars + 2 crc + nul */
	unsigned char *p;
	int bufsz, crc = 0;
	unsigned char trychar = 'C';
	unsigned char packetno = 1;
	int i, c, len = 0;
	int retry, retrans = MAXRETRANS;

	int idx = 0;

	if ((mode & M_START) != M_START) {
		packetno = recv_packetno;
		bufsz = recv_bufsz;
		crc = recv_crc;
		trychar = 0;
		c = _inbyte(DLY_1S);

		if (c == EOT) {
			flushinput();
			_outbyte(ACK);
			*eot = 1;
			return len; /* normal end */
		} else if (c == CAN) {
			if ((c = _inbyte(DLY_1S)) == CAN) {
				flushinput();
				_outbyte(ACK);
				return -1; /* canceled by remote */
			}
		}
		goto start_recv;
	} else {
		recv_packetno = 0;
		recv_bufsz = 0;
		recv_crc = 0;
	}

	for (;;) {
		for (retry = 0; retry < 16; ++retry) {
			if (trychar) {
				_outbyte(trychar);
			}
			c = _inbyte((DLY_1S) << 1);
			if ((c) >= 0) {
				switch (c) {
				case SOH:
					bufsz = 128;
					goto start_recv;
				case STX:
					bufsz = 1024;
					goto start_recv;
				case EOT:
					flushinput();
					_outbyte(ACK);
					*eot = 1;
					return len; /* normal end */
				case CAN:
					if ((c = _inbyte(DLY_1S)) == CAN) {
						flushinput();
						_outbyte(ACK);
						return -1; /* canceled by remote */
					}
					break;
				default:
					break;
				}
			}
		}
		if (trychar == 'C') {
			trychar = NAK;
			idx++;
			continue;
		}
		flushinput();
		_outbyte(CAN);
		_outbyte(CAN);
		_outbyte(CAN);
		return -2; /* sync error */

start_recv:
		if (trychar == 'C') {
			crc = 1;
		}
		trychar = 0;
		p = xbuff;
		*p++ = c;
		for (i = 0;  i < (bufsz + (crc ? 1 : 0) + 3); ++i) {
			if ((c = _inbyte(DLY_100MS)) < 0) {
				goto reject;
			}
			*p++ = c;
			if (xbuff[1] == dbg_pkg_no) {
				asm(" nop");
			}
		}

		if (xbuff[1] == (unsigned char)(~xbuff[2]) &&
			(xbuff[1] == packetno || xbuff[1] == (unsigned char)packetno - 1) &&
			check(crc, &xbuff[3], bufsz)) {
			if (xbuff[1] == packetno)	{
				register int count = destsz - len;

				if (count > bufsz) {
					count = bufsz;
				}
				if (count > 0) {
					memcpy(&dest[len], &xbuff[3], count);
					len += count;
				}
				++packetno;
				retrans = MAXRETRANS + 1;
			}
			if (--retrans <= 0) {
				flushinput();
				_outbyte(CAN);
				_outbyte(CAN);
				_outbyte(CAN);
				return -3; /* too many retry error */
			}
			_outbyte(ACK);

			if ((mode & M_FINAL) != M_FINAL && len >= destsz) {
				recv_packetno = packetno;
				recv_crc = crc;
				recv_bufsz = bufsz;
				*eot = 0;
				return len;
			}

			continue;
		}
reject:
		flushinput();
		_outbyte(NAK);
	}
}

// mode : 0, start, 1, middle, 2, end
static int xmit_packetno = 0;
static int xmit_crc = 0;
int xmodemTransmit(unsigned char *src, int srcsz, int mode)
{
	unsigned char xbuff[1030]; /* 1024 for XModem 1k + 3 head chars + 2 crc + nul */
	int bufsz, crc = -1;
	unsigned char packetno = 1;
	int i, c, len = 0;
	int retry;

	if ((mode & M_START) != M_START) {
		packetno = xmit_packetno;
		crc = xmit_crc;
		goto start_trans;
	} else {
		xmit_packetno = 0;
		xmit_crc = 0;
	}

	for (;;) {
		for (retry = 0; retry < 16; ++retry) {
			if ((c = _inbyte((DLY_1S) << 1)) >= 0) {
				switch (c) {
				case 'C':
					crc = 1;
					goto start_trans;
				case NAK:
					crc = 0;
					goto start_trans;
				case CAN:
					if ((c = _inbyte(DLY_1S)) == CAN) {
						_outbyte(ACK);
						flushinput();
						return -1; /* canceled by remote */
					}
					break;
				default:
					break;
				}
			}
		}
		_outbyte(CAN);
		_outbyte(CAN);
		_outbyte(CAN);
		flushinput();
		return -2; /* no sync */

		for (;;) {
start_trans:
			//xbuff[0] = SOH; bufsz = 128;
			xbuff[0] = STX;
			bufsz = 1024;
			xbuff[1] = packetno;
			xbuff[2] = ~packetno;
			c = srcsz - len;
			if (c > bufsz) {
				c = bufsz;
			}
			if (c > 0) {
				memset(&xbuff[3], 0, bufsz);
				if (c == 0) {
					xbuff[3] = CTRLZ;
				} else {
					memcpy(&xbuff[3], &src[len], c);
					if (c < bufsz) {
						xbuff[3 + c] = CTRLZ;
					}
				}
				if (crc) {
					unsigned short ccrc = crc16_ccitt(&xbuff[3], bufsz);
					xbuff[bufsz + 3] = (ccrc >> 8) & 0xFF;
					xbuff[bufsz + 4] = ccrc & 0xFF;
				} else {
					unsigned char ccks = 0;
					for (i = 3; i < bufsz + 3; ++i) {
						ccks += xbuff[i];
					}
					xbuff[bufsz + 3] = ccks;
				}
				for (retry = 0; retry < MAXRETRANS; ++retry) {
					for (i = 0; i < bufsz + 4 + (crc ? 1 : 0); ++i) {
						_outbyte(xbuff[i]);
					}
					if ((c = _inbyte(DLY_1S)) >= 0) {
						switch (c) {
						case ACK:
							++packetno;
							len += bufsz;
							goto start_trans;
						case CAN:
							if ((c = _inbyte(DLY_1S)) == CAN) {
								_outbyte(ACK);
								flushinput();
								return -1; /* canceled by remote */
							}
							break;
						case NAK:
						default:
							break;
						}
					}
				}
				_outbyte(CAN);
				_outbyte(CAN);
				_outbyte(CAN);
				flushinput();
				return -4; /* xmit error */
			} else {
				if ((mode & M_FINAL) == M_FINAL) {
					for (retry = 0; retry < 10; ++retry) {
						_outbyte(EOT);
						if ((c = _inbyte((DLY_1S) << 1)) == ACK) {
							break;
						}
					}
					flushinput();
					return (c == ACK) ? len : -5;
				} else {
					xmit_packetno = packetno;
					xmit_crc = crc;
					return len;
				}
			}
		}
	}
}
