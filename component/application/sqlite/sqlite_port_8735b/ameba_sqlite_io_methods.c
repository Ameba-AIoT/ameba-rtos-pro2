#ifdef SQLITE_OS_FREERTOS

#include "FreeRTOS.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <time.h>
#include "sqlite3.h"
#include "vfs.h"
#include "ameba_sqlite_common.h"

static int _ameba_io_read(sqlite3_file *file_id, void *pbuf, int cnt, sqlite3_int64 offset)
{
	ENTER();
	freertos_sqlite_file_t *p = (freertos_sqlite_file_t *)file_id;
	int rc = SQLITE_OK;
	int res = 0;

	CHK_ERR((res = fseek(p->fd, offset, SEEK_SET)) == 0,
			SQLITE_IOERR_SEEK,
			io_read_end,
			"[%s] fseek error with res:%d offset:%lld\r\n", __func__, res, offset);

	CHK_ERR((res = fread(pbuf, 1, cnt, p->fd)) >= 0,
			SQLITE_IOERR_READ,
			io_read_end,
			"[%s] fread error with res:%d \r\n", __func__, res);

io_read_end:
	LEAVE();
	return rc;
}

static int _ameba_io_write(sqlite3_file *file_id, const void *pbuf, int cnt, sqlite3_int64 offset)
{
	ENTER();
	freertos_sqlite_file_t *p = (freertos_sqlite_file_t *)file_id;
	int rc = SQLITE_OK;
	int res = 0;

	CHK_ERR((res = fseek(p->fd, offset, SEEK_SET)) == 0,
			SQLITE_IOERR_SEEK,
			io_write_end,
			"[%s] fseek error with res:%d offset:%lld\r\n", __func__, res, offset);

	CHK_ERR((res = fwrite(pbuf, 1, cnt, p->fd)) == cnt,
			SQLITE_IOERR_WRITE,
			io_write_end,
			"[%s] fwrite error with cnt:%d res:%d \r\n", __func__, cnt, res);

io_write_end:
	LEAVE();
	return rc;
}

static int _ameba_io_truncate(sqlite3_file *file_id, sqlite3_int64 size)
{
	ENTER();
	freertos_sqlite_file_t *p = (freertos_sqlite_file_t *)file_id;
	int rc = SQLITE_OK;
	int res = 0;

	CHK_ERR((res = ftruncate(p->fd, size)) == 0,
			SQLITE_IOERR_TRUNCATE,
			io_truncate_end,
			"[%s] ftruncate error with res:%d size:%lld\r\n", __func__, res, size);

io_truncate_end:
	LEAVE();
	return rc;
}

static int _ameba_io_sync(sqlite3_file *file_id, int flags)
{
	ENTER();
	freertos_sqlite_file_t *p = (freertos_sqlite_file_t *)file_id;
	int rc = SQLITE_OK;
	int res = 0;

	assert((flags & 0x0F) == SQLITE_SYNC_NORMAL
		   || (flags & 0x0F) == SQLITE_SYNC_FULL);

	CHK_ERR((res = fflush(p->fd)) == 0,
			SQLITE_IOERR_FSYNC,
			io_sync_end,
			"[%s] fflush error with res:%d \r\n", __func__, res);

io_sync_end:
	LEAVE();
	return rc;
}

static int _ameba_io_file_size(sqlite3_file *file_id, sqlite3_int64 *psize)
{
	ENTER();
	freertos_sqlite_file_t *p = (freertos_sqlite_file_t *)file_id;
	int rc = SQLITE_OK;
	int res = 0;

	assert(file_id);

	struct stat buf;
	CHK_ERR((res = stat(p->fileFullPath, &buf)) == 0,
			SQLITE_IOERR_FSTAT,
			io_file_size_end,
			"[%s] stat error with res:%d \r\n", __func__, res);

	*psize = buf.st_size;

	//printf("[%s] file size: %lld \r\n", __func__, *psize);

	/* When opening a zero-size database, the findInodeInfo() procedure
	** writes a single byte into that file in order to work around a bug
	** in the OS-X msdos filesystem.  In order to avoid problems with upper
	** layers, we need to report this file size as zero even though it is
	** really 1.   Ticket #3260.
	*/
	if (*psize == 1) {
		*psize = 0;
	}

io_file_size_end:
	LEAVE();
	return rc;
}

/*
** This routine checks if there is a RESERVED lock held on the specified
** file by this or any other process. If such a lock is held, set *pResOut
** to a non-zero value otherwise *pResOut is set to zero.  The return value
** is set to SQLITE_OK unless an I/O error occurs during lock checking.
*/
static int _ameba_io_check_reserved_lock(sqlite3_file *file_id, int *pResOut)
{
	ENTER();
	freertos_sqlite_file_t *p = (freertos_sqlite_file_t *)file_id;

	SemaphoreHandle_t mutex = p->mutex;
	int reserved = 0;

	/* Check if a thread in this process holds such a lock */
	if (p->eFileLock > SHARED_LOCK) {
		reserved = 1;
	}

	/* Otherwise see if some other process holds it. */
	if (!reserved) {
		if (xSemaphoreTakeRecursive(mutex, 0) != pdTRUE) {
			/* someone else has the lock when we are in NO_LOCK */
			reserved = (p->eFileLock < SHARED_LOCK);
		} else {
			/* we could have it if we want it */
			xSemaphoreGiveRecursive(mutex);
		}
	}

	*pResOut = reserved;
	LEAVE();
	return SQLITE_OK;
}

/*
** Lock the file with the lock specified by parameter eFileLock - one
** of the following:
**
**     (1) SHARED_LOCK
**     (2) RESERVED_LOCK
**     (3) PENDING_LOCK
**     (4) EXCLUSIVE_LOCK
**
** Sometimes when requesting one lock state, additional lock states
** are inserted in between.  The locking might fail on one of the later
** transitions leaving the lock state different from what it started but
** still short of its goal.  The following chart shows the allowed
** transitions and the inserted intermediate states:
**
**    UNLOCKED -> SHARED
**    SHARED -> RESERVED
**    SHARED -> (PENDING) -> EXCLUSIVE
**    RESERVED -> (PENDING) -> EXCLUSIVE
**    PENDING -> EXCLUSIVE
**
** Semaphore locks only really support EXCLUSIVE locks.  We track intermediate
** lock states in the sqlite3_file structure, but all locks SHARED or
** above are really EXCLUSIVE locks and exclude all other processes from
** access the file.
**
** This routine will only increase a lock.  Use the sqlite3OsUnlock()
** routine to lower a locking level.
*/
static int _ameba_io_lock(sqlite3_file *file_id, int eFileLock)
{
	ENTER();
	freertos_sqlite_file_t *p = (freertos_sqlite_file_t *)file_id;

	SemaphoreHandle_t mutex = p->mutex;
	int rc = SQLITE_OK;

	/* if we already have a lock, it is exclusive.
	** Just adjust level and punt on outta here. */
	if (p->eFileLock > NO_LOCK) {
		p->eFileLock = eFileLock;
		rc = SQLITE_OK;
		goto mutex_end_lock;
	}

	/* lock semaphore now but bail out when already locked. */
	if (xSemaphoreTakeRecursive(mutex, 0) != pdTRUE) {
		rc = SQLITE_BUSY;
		goto mutex_end_lock;
	}

	/* got it, set the type and return ok */
	p->eFileLock = eFileLock;

mutex_end_lock:
	LEAVE();
	return rc;
}

/*
** Lower the locking level on file descriptor pFile to eFileLock.  eFileLock
** must be either NO_LOCK or SHARED_LOCK.
**
** If the locking level of the file descriptor is already at or below
** the requested locking level, this routine is a no-op.
*/
static int _ameba_io_unlock(sqlite3_file *file_id, int eFileLock)
{
	ENTER();
	freertos_sqlite_file_t *p = (freertos_sqlite_file_t *)file_id;

	SemaphoreHandle_t mutex = p->mutex;
	int rc = SQLITE_OK;

	assert(eFileLock <= SHARED_LOCK);

	/* no-op if possible */
	if (p->eFileLock == eFileLock) {
		rc = SQLITE_OK;
		goto mutex_end_unlock;
	}

	/* shared can just be set because we always have an exclusive */
	if (eFileLock == SHARED_LOCK) {
		p->eFileLock = SHARED_LOCK;
		rc = SQLITE_OK;
		goto mutex_end_unlock;
	}

	/* no, really unlock. */
	xSemaphoreGiveRecursive(mutex);

	p->eFileLock = NO_LOCK;

mutex_end_unlock:
	LEAVE();
	return rc;
}

static int _ameba_io_close(sqlite3_file *file_id)
{
	ENTER();
	freertos_sqlite_file_t *p = (freertos_sqlite_file_t *)file_id;

	if (p->fd >= 0) {
		_ameba_io_unlock(file_id, NO_LOCK);
		vSemaphoreDelete(p->mutex);
		fclose(p->fd);
		p->fd = NULL;
	}

	LEAVE();
	return SQLITE_OK;
}

static int _ameba_fcntl_size_hint(sqlite3_file *file_id, sqlite3_int64 nByte)
{
	ENTER();
	freertos_sqlite_file_t *p = (freertos_sqlite_file_t *)file_id;
	int rc = SQLITE_OK;
	printf("[%s] nByte: %lld \r\n", __func__, nByte);

	if (p->szChunk > 0) {

		sqlite3_int64 nSize = ((nByte + p->szChunk - 1) / p->szChunk) * p->szChunk; /* Required file size */
		sqlite3_int64 file_size = 0;
		if (_ameba_io_file_size(file_id, &file_size)) {
			return SQLITE_IOERR_FSTAT;
		}

		printf("[%s] file_size: %lld, p->szChunk: %d \r\n", __func__, file_size, p->szChunk);

		if (nSize > file_size) {
			/* If the OS does not have posix_fallocate(), fake it. Write a
			** single byte to the last byte in each block that falls entirely
			** within the extended region. Then, if required, a single byte
			** at offset (nSize-1), to set the size of the file correctly.
			** This is a similar technique to that used by glibc on systems
			** that do not have a real fallocate() call.
			*/
			sqlite3_int64 iWrite = nSize - 1;   /* Next offset to write to */
			assert(iWrite >= file_size);

			rc = _ameba_io_write(file_id, "", 1, iWrite);
		}
	}

	LEAVE();
	return rc;
}

/*
** Information and control of an open file handle.
*/
static int _ameba_io_file_ctrl(sqlite3_file *file_id, int op, void *pArg)
{
	ENTER();
	freertos_sqlite_file_t *file = (freertos_sqlite_file_t *)file_id;

	switch (op) {
	case SQLITE_FCNTL_LOCKSTATE: {
		*(int *)pArg = file->eFileLock;
		return SQLITE_OK;
	}

	case SQLITE_LAST_ERRNO: {
		*(int *)pArg = 0;
		return SQLITE_OK;
	}

	case SQLITE_FCNTL_CHUNK_SIZE: {
		printf("[%s] SQLITE_FCNTL_CHUNK_SIZE \r\n", __func__);
		file->szChunk = *(int *)pArg;
		return SQLITE_OK;
	}

	case SQLITE_FCNTL_SIZE_HINT: {
		printf("[%s] SQLITE_FCNTL_SIZE_HINT \r\n", __func__);
		int rc;
		rc = _ameba_fcntl_size_hint(file_id, *(sqlite3_int64 *)pArg);
		return rc;
	}

	case SQLITE_FCNTL_VFSNAME: {
		*(char **)pArg = sqlite3_mprintf("%s", file->pvfs->zName);
		return SQLITE_OK;
	}

	case SQLITE_FCNTL_TEMPFILENAME: {
		char *zTFile = sqlite3_malloc(SQLITE_FREERTOS_MAX_PATHNAME);
		if (zTFile) {
			extern int _ameba_get_temp_name(sqlite3_vfs * pvfs, int nBuf, char *zBuf);
			_ameba_get_temp_name(file->pvfs, SQLITE_FREERTOS_MAX_PATHNAME, zTFile);
			*(char **)pArg = zTFile;
		}
		return SQLITE_OK;
	}
	}

	LEAVE();
	return SQLITE_NOTFOUND;
}

static int _ameba_io_sector_size(sqlite3_file *file_id)
{
	ENTER();
	LEAVE();
	return SQLITE_DEFAULT_SECTOR_SIZE;
}

static int _ameba_io_device_characteristics(sqlite3_file *file_id)
{
	ENTER();
	LEAVE();
	return 0;
}

/*
** If possible, return a pointer to a mapping of file fd starting at offset
** iOff. The mapping must be valid for at least nAmt bytes.
**
** If such a pointer can be obtained, store it in *pp and return SQLITE_OK.
** Or, if one cannot but no error occurs, set *pp to 0 and return SQLITE_OK.
** Finally, if an error does occur, return an SQLite error code. The final
** value of *pp is undefined in this case.
**
** If this function does return a pointer, the caller must eventually
** release the reference by calling unixUnfetch().
*/
static int _ameba_io_fetch(sqlite3_file *file_id, sqlite3_int64 iOff, int nAmt, void **pp)
{
	ENTER();
	*pp = 0;

	LEAVE();
	return SQLITE_OK;
}

/*
** If the third argument is non-NULL, then this function releases a
** reference obtained by an earlier call to unixFetch(). The second
** argument passed to this function must be the same as the corresponding
** argument that was passed to the unixFetch() invocation.
**
** Or, if the third argument is NULL, then this function is being called
** to inform the VFS layer that, according to POSIX, any existing mapping
** may now be invalid and should be unmapped.
*/
static int _ameba_io_unfetch(sqlite3_file *fd, sqlite3_int64 iOff, void *p)
{
	ENTER();
	LEAVE();
	return SQLITE_OK;
}

const sqlite3_io_methods _ameba_io_method = {
	.iVersion = 3,
	.xClose = _ameba_io_close,
	.xRead = _ameba_io_read,
	.xWrite = _ameba_io_write,
	.xTruncate = _ameba_io_truncate,
	.xSync = _ameba_io_sync,
	.xFileSize = _ameba_io_file_size,
	.xLock = _ameba_io_lock,
	.xUnlock = _ameba_io_unlock,
	.xCheckReservedLock = _ameba_io_check_reserved_lock,
	.xFileControl = _ameba_io_file_ctrl,
	.xSectorSize = _ameba_io_sector_size,
	.xDeviceCharacteristics = _ameba_io_device_characteristics,
	.xShmMap = NULL,
	.xShmLock = NULL,
	.xShmBarrier = NULL,
	.xShmUnmap = NULL,
	.xFetch = _ameba_io_fetch,
	.xUnfetch = _ameba_io_unfetch
};

#endif  /* SQLITE_OS_FREERTOS */
