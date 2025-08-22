#ifdef SQLITE_OS_FREERTOS

#include "FreeRTOS.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <time.h>
#include "sqlite3.h"
#include "vfs.h"
#include "sntp/sntp.h"
#include "ameba_sqlite_common.h"

static char g_ameba_interface_tag[16];

void set_ameba_sqlite_interface_tag(const char *tag)
{
	assert(strlen(tag) < sizeof(g_ameba_interface_tag));
	memset(g_ameba_interface_tag, 0, sizeof(g_ameba_interface_tag));
	snprintf(g_ameba_interface_tag, sizeof(g_ameba_interface_tag), "%s", tag);
}

/*
** Argument file_path points to a nul-terminated string containing a file path.
** If file_path is an absolute path, then it is copied as is into the output
** buffer. Otherwise, if it is a relative path, then the equivalent full
** path is written to the output buffer.
**
** This function assumes that paths are AmebaPro2 style. Specifically, that:
**
**   1. Path components are separated by a '/'. and
**   2. Full paths begin with a 'tag:/' character. It is required to set ameba vfs tag berfore starting sqlite.
*/
static int _ameba_vfs_fullpathname(sqlite3_vfs *pvfs, const char *file_path, int nOut, char *zOut)
{
	ENTER();
	assert(pvfs->mxPathname == SQLITE_FREERTOS_MAX_PATHNAME);
	assert(nOut > (strlen(file_path) + strlen(g_ameba_interface_tag)));

	memset(zOut, 0, nOut);
	if (strstr(file_path, g_ameba_interface_tag)) {
		sqlite3_snprintf(nOut, zOut, "%s", file_path);
	} else {
		sqlite3_snprintf(nOut, zOut, "%s%s", g_ameba_interface_tag, file_path);
	}

	LEAVE();
	return SQLITE_OK;
}

/*
** Create a temporary file name in zBuf.  zBuf must be allocated
** by the calling process and must be big enough to hold at least
** pVfs->mxPathname bytes.
*/
int _ameba_get_temp_name(sqlite3_vfs *pvfs, int nBuf, char *zBuf)
{
	ENTER();
	const unsigned char zChars[] = "abcdefghijklmnopqrstuvwxyz"
								   "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
								   "0123456789";
	unsigned int i, j;
	const char *zDir = "";

	/* Check that the output buffer is large enough for the temporary file
	** name. If it is not, return SQLITE_ERROR.
	*/
	if ((strlen(zDir) + strlen(SQLITE_TEMP_FILE_PREFIX) + 18) >= (size_t)nBuf) {
		return SQLITE_ERROR;
	}

	char zTmpFullname[SQLITE_FREERTOS_MAX_PATHNAME];
	do {
		memset(zBuf, 0, nBuf);
		sqlite3_snprintf(nBuf - 18, zBuf, "%s/"SQLITE_TEMP_FILE_PREFIX, zDir);
		j = (int)strlen(zBuf);
		sqlite3_randomness(15, &zBuf[j]);

		for (i = 0; i < 15; i++, j++) {
			zBuf[j] = (char)zChars[((unsigned char)zBuf[j]) % (sizeof(zChars) - 1)];
		}

		zBuf[j] = 0;
		zBuf[j + 1] = 0;

		memset(zTmpFullname, 0, sizeof(zTmpFullname));
		_ameba_vfs_fullpathname(pvfs, zBuf, sizeof(zTmpFullname), zTmpFullname);
		printf("[%s] generate tmp file name: %s \r\n", __func__, zTmpFullname);
	} while (access(zTmpFullname, F_OK) == 0);

	memcpy(zBuf, zTmpFullname, nBuf);

	LEAVE();
	return SQLITE_OK;
}

/*
** Invoke open().  Do so multiple times, until it either succeeds or
** fails for some reason other than EINTR.
**
** If the file creation mode "m" is 0 then set it to the default for
** SQLite.  The default is SQLITE_DEFAULT_FILE_PERMISSIONS (normally
** 0644) as modified by the system umask.  If m is not 0, then
** make the file creation mode be exactly m ignoring the umask.
**
** The m parameter will be non-zero only when creating -wal, -journal,
** and -shm files.  We want those files to have *exactly* the same
** permissions as their original database, unadulterated by the umask.
** In that way, if a database file is -rw-rw-rw or -rw-rw-r-, and a
** transaction crashes and leaves behind hot journals, then any
** process that is able to write to the database will also be able to
** recover the hot journals.
*/

static int _ameba_vfs_open(sqlite3_vfs *pvfs, const char *file_path, sqlite3_file *file_id, int flags, int *pOutFlags)
{
	ENTER();
	printf("[%s] open file name: %s \r\n", __func__, file_path);

	freertos_sqlite_file_t *p;
	FILE *fd;
	int eType = flags & 0xFFFFFF00;  /* Type of file to open */
	int rc = SQLITE_OK;            /* Function Return Code */
	char openMode[16];
	memset(openMode, 0, sizeof(openMode));

	int isExclusive  = (flags & SQLITE_OPEN_EXCLUSIVE);
	int isDelete     = (flags & SQLITE_OPEN_DELETEONCLOSE);
	int isCreate     = (flags & SQLITE_OPEN_CREATE);
	int isReadonly   = (flags & SQLITE_OPEN_READONLY);
	int isReadWrite  = (flags & SQLITE_OPEN_READWRITE);

	/* If argument zPath is a NULL pointer, this function is required to open
	** a temporary file. Use this buffer to store the file name in.
	*/
	char zTmpname[SQLITE_FREERTOS_MAX_PATHNAME];
	memset(zTmpname, 0, sizeof(zTmpname));

	p = (freertos_sqlite_file_t *)file_id;

	/* Check the following statements are true:
	**
	**   (a) Exactly one of the READWRITE and READONLY flags must be set, and
	**   (b) if CREATE is set, then READWRITE must also be set, and
	**   (c) if EXCLUSIVE is set, then CREATE must also be set.
	**   (d) if DELETEONCLOSE is set, then CREATE must also be set.
	*/
	assert((isReadonly == 0 || isReadWrite == 0) && (isReadWrite || isReadonly));
	assert(isCreate == 0 || isReadWrite);
	assert(isExclusive == 0 || isCreate);
	assert(isDelete == 0 || isCreate);

	/* The main DB, main journal, WAL file and master journal are never
	** automatically deleted. Nor are they ever temporary files.  */
	assert((!isDelete && file_path) || eType != SQLITE_OPEN_MAIN_DB);
	assert((!isDelete && file_path) || eType != SQLITE_OPEN_MAIN_JOURNAL);
	assert((!isDelete && file_path) || eType != SQLITE_OPEN_MASTER_JOURNAL);
	assert((!isDelete && file_path) || eType != SQLITE_OPEN_WAL);

	/* Assert that the upper layer has set one of the "file-type" flags. */
	assert(eType == SQLITE_OPEN_MAIN_DB      || eType == SQLITE_OPEN_TEMP_DB
		   || eType == SQLITE_OPEN_MAIN_JOURNAL || eType == SQLITE_OPEN_TEMP_JOURNAL
		   || eType == SQLITE_OPEN_SUBJOURNAL   || eType == SQLITE_OPEN_MASTER_JOURNAL
		   || eType == SQLITE_OPEN_TRANSIENT_DB || eType == SQLITE_OPEN_WAL
		  );

	/* Database filenames are double-zero terminated if they are not
	** URIs with parameters.  Hence, they can always be passed into
	** sqlite3_uri_parameter(). */
	assert((eType != SQLITE_OPEN_MAIN_DB) || (flags & SQLITE_OPEN_URI) || file_path[strlen(file_path) + 1] == 0);

	memset(p, 0, sizeof(freertos_sqlite_file_t));
	if (!file_path) {
		rc = _ameba_get_temp_name(pvfs, sizeof(zTmpname), zTmpname);
		if (rc != SQLITE_OK) {
			goto vfs_open_end;
		}
		file_path = zTmpname;
		printf("[%s] open tmp file name: %s \r\n", __func__, file_path);

		/* Generated temporary filenames are always double-zero terminated
		** for use by sqlite3_uri_parameter(). */
		assert(file_path[strlen(file_path) + 1] == 0);
	}

	/* Determine the value of the flags parameter passed to POSIX function
	** open(). These must be calculated even if open() is not called, as
	** they may be stored as part of the file handle and used by the
	** 'conch file' locking functions later on.  */

	int already_exist = access(file_path, F_OK) == 0 ? 1 : 0;
	if (already_exist) {
		printf("[%s] %s already exist \r\n", __func__, file_path);
	}

	int curr = 0;
	if (isCreate && !already_exist) {
		curr += snprintf(&openMode[curr], sizeof(openMode) - curr, "%s", "w");
	}
	if (isReadonly) {
		curr += snprintf(&openMode[curr], sizeof(openMode) - curr, "%s", "r");
	}
	if (isReadWrite) {
		curr += snprintf(&openMode[curr], sizeof(openMode) - curr, "%s", "r+");
	}
	if (isExclusive) {
		CHK_ERR(already_exist,
				SQLITE_NOTFOUND,
				vfs_open_end,
				"[%s] file not found \r\n", __func__);
	}

	//printf("[%s] open file with mode: %s \r\n", __func__, openMode);
	fd = fopen(file_path, openMode);
	if (!fd) {
		/* Failed to open the file for read/write access. Try read-only. */
		printf("[%s] failed to open the file for read/write access. Try read-only. \r\n", __func__);
		flags &= ~(SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE);
		flags |= SQLITE_OPEN_READONLY;
		fd = fopen(file_path, "r");
		CHK_ERR(fd != NULL,
				SQLITE_CANTOPEN,
				vfs_open_end,
				"[%s] fail to open file %s \r\n", __func__, file_path);
	}

	if (pOutFlags) {
		*pOutFlags = flags;
	}

	if (isDelete) {
		remove(file_path);
	}

	p->fd = fd;
	extern const sqlite3_io_methods _ameba_io_method;
	p->pMethod = &_ameba_io_method;
	p->eFileLock = NO_LOCK;
	p->szChunk = 0;
	p->pvfs = pvfs;
	p->mutex = xSemaphoreCreateRecursiveMutex();
	CHK_ERR(p->mutex != NULL,
			SQLITE_NOMEM,
			vfs_open_end,
			"[%s] fail to create mutex, OOM. \r\n", __func__);
	snprintf(p->fileFullPath, sizeof(p->fileFullPath), "%s", file_path);

vfs_open_end:
	LEAVE();
	return rc;
}

static int _ameba_vfs_delete(sqlite3_vfs *pvfs, const char *file_path, int syncDir)
{
	ENTER();
	printf("[%s] delete file name: %s \r\n", __func__, file_path);

	int rc = SQLITE_OK;
	CHK_ERR(remove(file_path) != -1,
			SQLITE_ERROR,
			vfs_delete_end,
			"[%s] fail to delete file %s \r\n", __func__, file_path);

	if ((syncDir & 1) != 0) {
		//not required
	}

vfs_delete_end:
	LEAVE();
	return rc;
}

static int _ameba_vfs_access(sqlite3_vfs *pvfs, const char *file_path, int flags, int *pResOut)
{
	ENTER();
	//printf("[%s] access file name: %s \r\n", __func__, file_path);

	int amode = 0;

	//printf("[%s] access flag: %d \r\n", __func__, flags);
	switch (flags) {
	case SQLITE_ACCESS_EXISTS:
		amode = F_OK;
		break;

	case SQLITE_ACCESS_READWRITE:
		amode = W_OK | R_OK;
		break;

	case SQLITE_ACCESS_READ:
		amode = R_OK;
		break;

	default:
		//
		return -1;
	}

	int ret = access(file_path, amode);
	//printf("[%s] access result: %d \r\n", __func__, ret);
	*pResOut = (ret == 0);

	if (flags == SQLITE_ACCESS_EXISTS && *pResOut) {
		struct stat buf;

		if (0 == stat(file_path, &buf) && (buf.st_size == 0)) {
			*pResOut = 0;
		}
	}

	LEAVE();
	return SQLITE_OK;
}

static int _ameba_vfs_randomness(sqlite3_vfs *pvfs, int nByte, char *zOut)
{
	ENTER();
	assert((size_t)nByte >= (sizeof(time_t) + sizeof(int)));

	memset(zOut, 0, nByte);
	{
		int i;
		char tick8, tick16;

		tick8 = (char)xTaskGetTickCount();
		tick16 = (char)(xTaskGetTickCount() >> 8);

		for (i = 0; i < nByte; i++) {
			zOut[i] = (char)(i ^ tick8 ^ tick16);
			tick8 = zOut[i];
			tick16 = ~(tick8 ^ tick16);
		}
	}

	LEAVE();
	return nByte;
}

static int _ameba_vfs_sleep(sqlite3_vfs *pvfs, int microseconds)
{
	ENTER();
	int millisecond = (microseconds + 999) / 1000;

	vTaskDelay(millisecond / portTICK_PERIOD_MS);

	LEAVE();
	return millisecond * 1000;
}

static int _ameba_vfs_current_time_int64(sqlite3_vfs *pvfs, sqlite3_int64 *pnow)
{
	ENTER();
	static const sqlite3_int64 freertosEpoch = 24405875 * (sqlite3_int64)8640000;

	struct timeval tv;
	gettimeofday(&tv, NULL);

	*pnow = freertosEpoch + (sqlite3_int64)(tv.tv_sec) * 1000 + (sqlite3_int64)(tv.tv_usec) / 1000;

	LEAVE();
	return SQLITE_OK;
}

static int _ameba_vfs_current_time(sqlite3_vfs *pvfs, double *pnow)
{
	ENTER();
	sqlite3_int64 t = 0;
	_ameba_vfs_current_time_int64(0, &t);

	*pnow = t / 86400000.0;

	LEAVE();
	return SQLITE_OK;
}

static int _ameba_vfs_get_last_error(sqlite3_vfs *pvfs, int nBuf, char *zBuf)
{
	ENTER();
	LEAVE();
	return 0;
}


static int _ameba_vfs_set_system_call(sqlite3_vfs *pvfs, const char *file_path, sqlite3_syscall_ptr pfn)
{
	ENTER();
	LEAVE();
	return SQLITE_NOTFOUND;
}

static sqlite3_syscall_ptr _ameba_vfs_get_system_call(sqlite3_vfs *pvfs, const char *file_path)
{
	ENTER();
	LEAVE();
	return 0;
}

static const char *_ameba_vfs_next_system_call(sqlite3_vfs *pvfs, const char *file_path)
{
	ENTER();
	LEAVE();
	return 0;
}

/*
** Initialize and deinitialize the operating system interface.
*/
SQLITE_API int sqlite3_os_init(void)
{
	ENTER();

	static sqlite3_vfs _ameba_vfs = {
		.iVersion =         3,                                  /* iVersion */
		.szOsFile =         sizeof(freertos_sqlite_file_t),     /* szOsFile */
		.mxPathname =       SQLITE_FREERTOS_MAX_PATHNAME,              /* mxPathname */
		.pNext =            NULL,                               /* pNext */
		.zName =            "AmebaVFS",                         /* zName */
		.pAppData =         NULL,                               /* pAppData */
		.xOpen =            _ameba_vfs_open,                 /* xOpen */
		.xDelete =          _ameba_vfs_delete,               /* xDelete */
		.xAccess =          _ameba_vfs_access,               /* xAccess */
		.xFullPathname =    _ameba_vfs_fullpathname,         /* xFullPathname */
		.xDlOpen =          NULL,                               /* xDlOpen */
		.xDlError =         NULL,                               /* xDlError */
		.xDlSym =           NULL,                               /* xDlSym */
		.xDlClose =         NULL,                               /* xDlClose */
		.xRandomness =      _ameba_vfs_randomness,           /* xRandomness */
		.xSleep =           _ameba_vfs_sleep,                /* xSleep */
		.xCurrentTime =     _ameba_vfs_current_time,         /* xCurrentTime */
		.xGetLastError =    _ameba_vfs_get_last_error,       /* xGetLastError */
		.xCurrentTimeInt64 = _ameba_vfs_current_time_int64,  /* xCurrentTimeInt64 */
		.xSetSystemCall =   _ameba_vfs_set_system_call,      /* xSetSystemCall */
		.xGetSystemCall =   _ameba_vfs_get_system_call,      /* xGetSystemCall */
		.xNextSystemCall =  _ameba_vfs_next_system_call,     /* xNextSystemCall */
	};

	sqlite3_vfs_register(&_ameba_vfs, 1);

	LEAVE();
	return SQLITE_OK;
}

SQLITE_API int sqlite3_os_end(void)
{
	ENTER();
	LEAVE();
	return SQLITE_OK;
}

#endif  /* SQLITE_OS_FREERTOS */

