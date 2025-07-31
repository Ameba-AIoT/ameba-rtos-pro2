#ifndef _AMEBA_SQLITE_CONFIG_H_
#define _AMEBA_SQLITE_CONFIG_H_

/*
** Define various macros that are missing from some systems.
*/
#define NO_LOCK         0
#define SHARED_LOCK     1
#define RESERVED_LOCK   2
#define PENDING_LOCK    3
#define EXCLUSIVE_LOCK  4

#ifndef SQLITE_DEFAULT_SECTOR_SIZE
#define SQLITE_DEFAULT_SECTOR_SIZE 4096
#endif

#ifndef SQLITE_TEMP_FILE_PREFIX
# define SQLITE_TEMP_FILE_PREFIX "etilqs_"
#endif

#define SQLITE_FREERTOS_MAX_PATHNAME    256

#define AMEBA_DEBUG 0
#if AMEBA_DEBUG
#define ENTER() do{ printf("Enter %s \r\n", __func__);} while(0)
#define LEAVE() do{ printf("Leave %s \r\n", __func__);} while(0)
#else
#define ENTER() do{} while(0)
#define LEAVE() do{} while(0)
#endif

#define CHK_ERR(condition, errRet, endFlag, errorMessage, ...)   \
    do {                                                \
        if (!(condition)) {                             \
            rc = (errRet);                              \
            printf(errorMessage, ##__VA_ARGS__);        \
            goto endFlag;                               \
        }                                               \
    } while (0)

typedef struct freertos_sqlite_file_s {
	sqlite3_io_methods const *pMethod;
	sqlite3_vfs *pvfs;
	FILE *fd;
	char fileFullPath[SQLITE_FREERTOS_MAX_PATHNAME];
	int eFileLock;
	int szChunk;
	SemaphoreHandle_t mutex;
} freertos_sqlite_file_t;

void set_ameba_sqlite_interface_tag(const char *tag);

#endif /* _AMEBA_SQLITE_CONFIG_H_ */
