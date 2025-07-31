/* taken from nacl-20110221, from curvecp/load.c */
//#include <unistd.h>
#include "readall.h"
#include "open.h"
#include "e.h"
#include "load.h"

#include <string.h>
#include "sshcrypto.h"

extern unsigned char tinyssh_sign_publickey[crypto_sign_ed25519_PUBLICKEYBYTES];
extern unsigned char tinyssh_sign_secretkey[crypto_sign_ed25519_SECRETKEYBYTES];

int load(const char *fn, void *x, long long xlen)
{
#if 0
	int fd;
	int r;
	fd = open_read(fn);
	if (fd == -1) {
		return -1;
	}
	r = readall(fd, x, xlen);
	close(fd);
	return r;
#else
	if (strcmp(fn, sshcrypto_keys[0].sign_publickeyfilename) == 0) {
		memcpy(x, tinyssh_sign_publickey, crypto_sign_ed25519_PUBLICKEYBYTES > xlen ? xlen : crypto_sign_ed25519_PUBLICKEYBYTES);
		return 0;
	}
	if (strcmp(fn, sshcrypto_keys[0].sign_secretkeyfilename) == 0) {
		memcpy(x, tinyssh_sign_secretkey, crypto_sign_ed25519_SECRETKEYBYTES > xlen ? xlen : crypto_sign_ed25519_SECRETKEYBYTES);
		return 0;
	}
	return -1;
#endif
}
