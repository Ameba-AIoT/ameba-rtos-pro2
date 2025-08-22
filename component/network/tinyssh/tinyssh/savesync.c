/* taken from nacl-20110221, from curvecp/savesync.c */
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>
//#include <unistd.h>
#include "open.h"
#include "savesync.h"
#include "writeall.h"

#include <string.h>
#include "sshcrypto.h"

extern unsigned char tinyssh_sign_publickey[crypto_sign_ed25519_PUBLICKEYBYTES];
extern unsigned char tinyssh_sign_secretkey[crypto_sign_ed25519_SECRETKEYBYTES];

#if 0
static int writesync(int fd, const void *x, long long xlen)
{
	if (writeall(fd, x, xlen) == -1) {
		return -1;
	}
	return fsync(fd);
}
#endif
int savesync(const char *fn, const void *x, long long xlen)
{
#if 0
	int fd;
	int r;
	fd = open_write(fn);
	if (fd == -1) {
		return -1;
	}
	r = writesync(fd, x, xlen);
	close(fd);
	return r;
#else
	if (strcmp(fn, sshcrypto_keys[0].sign_publickeyfilename) == 0) {
		memcpy(tinyssh_sign_publickey, x, xlen > crypto_sign_ed25519_PUBLICKEYBYTES ? crypto_sign_ed25519_PUBLICKEYBYTES : xlen);
		return 0;
	}
	if (strcmp(fn, sshcrypto_keys[0].sign_secretkeyfilename) == 0) {
		memcpy(tinyssh_sign_secretkey, x, xlen > crypto_sign_ed25519_SECRETKEYBYTES ? crypto_sign_ed25519_SECRETKEYBYTES : xlen);
		return 0;
	}
	return -1;
#endif
}
