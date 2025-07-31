/*
20140214
Jan Mojzis
Public domain.
*/

//#include <sys/types.h>
//#include <sys/wait.h>
//#include <sys/stat.h>
//#include <unistd.h>
//#include <pwd.h>
#include "str.h"
#include "open.h"
#include "dropuidgid.h"
#include "global.h"
#include "e.h"
#include "log.h"
#include "sshcrypto.h"
#include "getln.h"
#include "bug.h"
#include "limit.h"
#include "subprocess.h"

#include <string.h>

static int findnameandkey(const char *keyname, const char *key, char *x)
{

	if (!str_start(x, keyname)) {
		return 0;
	}
	x += str_len(keyname);
	if (*x != ' ') {
		return 0;
	}
	x += 1;
	if (!str_start(x, key)) {
		return 0;
	}
	x += str_len(key);
	if (*x == ' ') {
		return 1;
	}
	if (*x == '\n') {
		return 1;
	}
	if (*x == '\r') {
		return 1;
	}
	if (*x == 0) {
		return 1;
	}
	return 0;
}

static void check(uid_t uid, const char *d, const char *f, long long *err)
{

#if 0
	struct stat st;
	int e = 0;

	if (f) {
		if (stat(f, &st) == -1) {
			log_w4("auth: unable to stat file: ", d, "/", f);
			e = 1;
		}
		if (e == 0 && !S_ISREG(st.st_mode)) {
			errno = EACCES;
			log_w4("auth: non-regular file: ", d, "/", f);
			e = 1;
		}
	} else {
		if (stat(d, &st) == -1) {
			log_w4("auth: unable to stat directory: ", d, "/", f);
			e = 1;
		}
		if (e == 0 && !S_ISDIR(st.st_mode)) {
			errno = ENOTDIR;
			log_w4("auth: unable to stat directory: ", d, "/", f);
			e = 1;
		}
	}
	if (e == 0 && (st.st_mode & 022) != 0) {
		errno = EACCES;
		log_w4("auth: bad mode: directory writable by group or others: ", d, "/", f);
		e = 1;
	}
	if (e == 0 && st.st_uid != uid && st.st_uid != 0) {
		errno = EACCES;
		log_w4("auth: bad owner: ", d, "/", f);
		e = 1;
	}

	if (e) {
		*err = 1;
	} else {
		log_d4("auth: path: ok: ", d, "/", f);
	}
#endif
}

int subprocess_auth_checkpath_(char *path, long long pathlen, uid_t uid)
{

#if 0
	long long err = 0, j, i;

	if (pathlen <= 16) {
		return 0;
	}
	if (!getcwd(path, pathlen)) {
		return 0;
	}
	for (i = 0; i < pathlen; ++i) if (path[i] == 0) {
			break;
		}
	if (i == pathlen) {
		return 0;
	}

	check(uid, path, "authorized_keys", &err);

	do {
		check(uid, path, 0, &err);
		for (j = i; j >= 0; --j) {
			if (path[j] == '/') {
				path[j] = 0;
				i = j;
				break;
			}
		}
	} while (j > 0);
	return (err == 0);
#else
	return 1;
#endif
}

int subprocess_auth_authorizedkeys_(const char *keyname, const char *key, const char *dir, char *buf, long long bufmax)
{
#if 0
	int fd = -1;
	int r;

	fd = open_read("authorized_keys");
	if (fd == -1) {
		log_w3("auth: unable to open file: ", dir, "/.ssh/authorized_keys");
		return 0;
	}

	do {
		r = getln(fd, buf, bufmax);
		if (r == -1) {
			log_w3("auth: unable to read from file ", dir, "/.ssh/authorized_keys");
			return 0;
		}
		if (findnameandkey(keyname, key, buf)) {
			return 1;    /* authorized */
		}
	} while (r > 0);
	log_w1("auth: unable to authorize using authorized_keys: key not found");
	return 0;
#else
	return 1;
#endif
}

/*
The 'subprocess_auth' is used for authorization using
~/.ssh/authorized_keys file.
*/
int subprocess_auth(const char *account, const char *keyname, const char *key)
{
#if 0
	pid_t pid;
	int status;

	pid = fork();
	if (pid == -1) {
		return -1;
	}
	if (pid == 0) {

#define buf global_bspace2 /* reusing global buffer */
		struct passwd *pw;

		if (!account || !keyname || !key) {
			bug_inval();
		}
		if (sshcrypto_sign_BASE64PUBLICKEYMIN > str_len(key) + 1) {
			bug_inval();
		}

		/* drop privileges */
		pw = getpwnam(account);
		if (!pw) {
			log_w3("auth: account ", account, ": not exist");
			global_die(111);
		}
		if (!dropuidgid(pw->pw_name, pw->pw_uid, pw->pw_gid)) {
			log_w2("auth: unable to drop privileges to account ", account);
			global_die(111);
		}

		/* change directory to ~/.ssh */
		if (chdir(pw->pw_dir) == -1) {
			log_w2("auth: unable to change directory to ", pw->pw_dir);
			global_die(111);
		}
		if (chdir(".ssh") == -1) {
			log_w3("auth: unable to change directory to ", pw->pw_dir, "/.ssh");
			global_die(111);
		}

		/* authorization starts here */
		if (!subprocess_auth_checkpath_((char *)buf, sizeof buf, pw->pw_uid)) {
			global_die(111);
		}
		if (!subprocess_auth_authorizedkeys_(keyname, key, pw->pw_dir, (char *)buf, sizeof buf)) {
			global_die(111);
		}
		/* authorization ends here */

		global_die(0);
	}

	while (waitpid(pid, &status, 0) != pid) {};
	if (!WIFEXITED(status)) {
		return -1;
	}
	return WEXITSTATUS(status);
#elif 1
#define buf global_bspace2 /* reusing global buffer */
	if (!account || !keyname || !key) {
		bug_inval();
	}
	if (sshcrypto_sign_BASE64PUBLICKEYMIN > str_len(key) + 1) {
		bug_inval();
	}

	/* authorization starts here */
	extern const char *tinyssh_client_account;
	extern const char *tinyssh_client_keyname;
	extern const char *tinyssh_client_publickey;
	if (!((strlen(account) == strlen(tinyssh_client_account)) && (strcmp(account, tinyssh_client_account) == 0))) {
		global_die(111);
	}
	if (!((strlen(keyname) == strlen(tinyssh_client_keyname)) && (strcmp(keyname, tinyssh_client_keyname) == 0))) {
		global_die(111);
	}
	if (!((strlen(key) == strlen(tinyssh_client_publickey)) && (strcmp(key, tinyssh_client_publickey) == 0))) {
		global_die(111);
	}
	/* authorization ends here */

	return 0;
#else
	log_f7(__func__, " account=", account, ", keyname=", keyname, ", key=", key);
	return 0;
#endif
}
