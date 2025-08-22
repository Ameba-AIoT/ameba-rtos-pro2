/******************************************************************************
*
* Copyright(c) 2007 - 2022 Realtek Corporation. All rights reserved.
*
******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "osdep_service.h"
#include "sys_api.h"
#include "vfs.h"
#include "sqlite3.h"
#include "ameba_sqlite_common.h"

/* configuration */
#define SQLITE_USE_REALTIME     0

enum sqlite_interface {
	SQLITE_INTERFACE_SD = 0,    //Fatfs
	SQLITE_INTERFACE_FLASH,     //LittleFS:Nand/Nor, Fatfs:Nor
	SQLITE_INTERFACE_RAM,       //Fatfs
};

static int exec_msg_callback(void *param, int argc, char **argv, char **azColName)
{
	char *echo = (char *)param;
	printf("%s \r\n", echo);

	int i;
	for (i = 0; i < argc; i++) {
		printf("%s = %s\r\n", azColName[i], argv[i] ? argv[i] : "NULL");
	}
	printf("\r\n");
	return 0;
}

static int select_query_by_prepare_step_finalize(sqlite3 *db, const char *sql)
{
	int rc = SQLITE_OK;

	sqlite3_stmt *stmt;
	rc = sqlite3_prepare(db, sql, -1, &stmt, NULL);
	if (rc) {
		printf("sqlite3_prepare error: %d \r\n", rc);
		goto sqlite_query_end;
	}

	printf("SELECT query by prepare, step, finalize... \r\n");
	while (sqlite3_step(stmt) != SQLITE_DONE) {

		int num_cols = sqlite3_column_count(stmt);
		for (int i = 0; i < num_cols; i++) {
			switch (sqlite3_column_type(stmt, i)) {
			case (SQLITE3_TEXT):
				printf("%-10s ", sqlite3_column_text(stmt, i));
				break;
			case (SQLITE_INTEGER):
				printf("%-6d ", sqlite3_column_int(stmt, i));
				break;
			case (SQLITE_FLOAT):
				printf("%-6g ", sqlite3_column_double(stmt, i));
				break;
			default:
				break;
			}
		}
		printf("\r\n");
	}
	printf("\r\n");

	rc = sqlite3_finalize(stmt);
	if (rc) {
		printf("sqlite3_finalize error: %d \r\n", rc);
		goto sqlite_query_end;
	}

sqlite_query_end:
	return rc;
}

static int sqlite_test(void)
{
	printf("=== START SQLite TEST === \r\n");

	sqlite3_initialize();

	sqlite3 *db = NULL;
	char *zErrMsg = NULL;
	int rc;
	const char *sql;
	const char *data = "Callback function called";
	const char *database_name = "test.db";

	/* sqlite lib version */
	printf("SQLite version: %s \r\n", sqlite3_libversion());

	/* connect to an existing database. If the database does not exist, then it will be created and finally a database object will be returned. */
	printf("Connect or create a database... \r\n");
	rc = sqlite3_open(database_name, &db);
	if (rc) {
		printf("Can't open database: %s\r\n", sqlite3_errmsg(db));
		goto sqlite_close;
	} else {
		printf("Opened database successfully\r\n");
	}

	/* Create a Table */
	printf("Create a Table... \r\n");
	sql = "DROP TABLE IF EXISTS COMPANY; " \
		  "CREATE TABLE COMPANY("  \
		  "ID INT PRIMARY KEY     NOT NULL," \
		  "NAME           TEXT    NOT NULL," \
		  "AGE            INT     NOT NULL," \
		  "ADDRESS        CHAR(50)," \
		  "SALARY         REAL );";
	/* Execute SQL statement */
	rc = sqlite3_exec(db, sql, exec_msg_callback, (void *)data, &zErrMsg);
	if (rc != SQLITE_OK) {
		printf("SQL error: %s\n", zErrMsg);
		goto sqlite_close;
	} else {
		printf("Table created successfully\r\n");
	}

	/* INSERT Operation */
	printf("Test INSERT Operation... \r\n");
	sql = "INSERT INTO COMPANY (ID,NAME,AGE,ADDRESS,SALARY) "  \
		  "VALUES (1, 'Paul', 32, 'California', 20000.00 ); " \
		  "INSERT INTO COMPANY (ID,NAME,AGE,ADDRESS,SALARY) "  \
		  "VALUES (2, 'Allen', 25, 'Texas', 15000.00 ); "     \
		  "INSERT INTO COMPANY (ID,NAME,AGE,ADDRESS,SALARY)" \
		  "VALUES (3, 'Teddy', 23, 'Norway', 20000.00 );" \
		  "INSERT INTO COMPANY (ID,NAME,AGE,ADDRESS,SALARY)" \
		  "VALUES (4, 'Mark', 25, 'Rich-Mond ', 65000.00 );";
	/* Execute SQL statement */
	rc = sqlite3_exec(db, sql, exec_msg_callback, (void *)data, &zErrMsg);
	if (rc != SQLITE_OK) {
		printf("SQL error: %s\r\n", zErrMsg);
		goto sqlite_close;
	} else {
		printf("Records created successfully\r\n");
	}

	/* SELECT Operation */
	printf("Test SELECT Operation... \r\n");
	sql = "SELECT * from COMPANY";
	/* Execute SQL statement */
	rc = sqlite3_exec(db, sql, exec_msg_callback, (void *)data, &zErrMsg);
	if (rc != SQLITE_OK) {
		printf("SQL error: %s\r\n", zErrMsg);
		goto sqlite_close;
	} else {
		printf("Operation done successfully\r\n");
	}

	/* SELECT Operation by prepare, step, finalize */
	rc = select_query_by_prepare_step_finalize(db, sql);
	if (rc != SQLITE_OK) {
		printf("select_query_by_prepare_step_finalize fail \r\n");
		goto sqlite_close;
	} else {
		printf("Operation done successfully\r\n");
	}

	/* UPDATE Operation */
	printf("Test UPDATE Operation... \r\n");
	sql = "UPDATE COMPANY set SALARY = 25000.00 where ID=1; " \
		  "SELECT * from COMPANY";
	/* Execute SQL statement */
	rc = sqlite3_exec(db, sql, exec_msg_callback, (void *)data, &zErrMsg);
	if (rc != SQLITE_OK) {
		printf("SQL error: %s\r\n", zErrMsg);
		goto sqlite_close;
	} else {
		printf("Operation done successfully\r\n");
	}

	/* DELETE Operation */
	printf("Test DELETE Operation... \r\n");
	sql = "DELETE from COMPANY where ID=2; " \
		  "SELECT * from COMPANY";
	/* Execute SQL statement */
	rc = sqlite3_exec(db, sql, exec_msg_callback, (void *)data, &zErrMsg);
	if (rc != SQLITE_OK) {
		printf("SQL error: %s\r\n", zErrMsg);
		goto sqlite_close;
	} else {
		printf("Operation done successfully\r\n");
	}

	/* DROP table */
	printf("Test DROP table... \r\n");
	sql = "DROP TABLE COMPANY; " ;
	/* Execute SQL statement */
	rc = sqlite3_exec(db, sql, exec_msg_callback, (void *)data, &zErrMsg);
	if (rc != SQLITE_OK) {
		printf("SQL error: %s\r\n", zErrMsg);
		goto sqlite_close;
	} else {
		printf("Operation done successfully\r\n");
	}

sqlite_close:
	if (zErrMsg) {
		sqlite3_free(zErrMsg);
		zErrMsg = NULL;
	}
	if (db) {
		printf("close sqlite db \r\n");
		sqlite3_close(db);
		db = NULL;
	}

	printf("=== FINISH SQLite TEST === \r\n");

	if (rc != SQLITE_OK) {
		return -1;
	}
	return 0;
}

#if SQLITE_USE_REALTIME
#include "wifi_conf.h"
#include "lwip_netconf.h"
#define wifi_wait_time 500 //Here we wait 5 second to wiat the fast connect 
static void wifi_common_init(void)
{
	uint32_t wifi_wait_count = 0;

	while (!((wifi_get_join_status() == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
		vTaskDelay(10);
		wifi_wait_count++;
		if (wifi_wait_count == wifi_wait_time) {
			printf("\r\nuse ATW0, ATW1, ATWC to make wifi connection\r\n");
			printf("wait for wifi connection...\r\n");
		}
	}
}
#endif

static void prv_set_interface_tag_in_sqlite_vfs(int idx)
{
	const char *tag = NULL;
	if (idx == SQLITE_INTERFACE_SD) {
		tag = "sd:/";
	} else if (idx == SQLITE_INTERFACE_FLASH) {
		tag = "lfs:/";
	} else if (idx == SQLITE_INTERFACE_RAM) {
		tag = "ram:/";
	}
	assert(tag != NULL);
	set_ameba_sqlite_interface_tag(tag);
}

static void example_sqlite_thread(void *para)
{
#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif

#if SQLITE_USE_REALTIME
	/* check wifi connected */
	wifi_common_init();
	/* check sntp inited */
	sntp_init();
	while (time(NULL) < 100ULL) {
		vTaskDelay(200 / portTICK_PERIOD_MS);
		printf("waiting sntp get correct time... \r\n");
	}
#endif

	/* init VFS on Ameba */
	vfs_init(NULL);
	vfs_user_register("sd", VFS_FATFS, VFS_INF_SD);
	vfs_user_register("lfs", VFS_LITTLEFS, 0);
	vfs_user_register("ram", VFS_FATFS, VFS_INF_RAM);

	int test_cnt = 3, pass_cnt = 0;
	for (int idx = 0; idx < test_cnt; idx++) {
		/* set sqlite path name conversion tag */
		prv_set_interface_tag_in_sqlite_vfs(idx);

		/* do test */
		int ret = sqlite_test();
		if (ret == 0) {
			printf("run sqlite test successfully [%d] \r\n", idx + 1);
			pass_cnt++;
		} else {
			printf("run sqlite test fail [%d] \r\n", idx + 1);
		}
	}

	/* deinit VFS on Ameba */
	vfs_user_unregister("sd", VFS_FATFS, VFS_INF_SD);
	vfs_user_unregister("lfs", VFS_LITTLEFS, 0);
	vfs_user_unregister("ram", VFS_FATFS, VFS_INF_RAM);
	/* deinit VFS */
	vfs_deinit(NULL);

	printf("========================================= \r\n");
	printf("TEST RESULT (TOTAL: %d): %d PASS, %d FAIL \r\n", test_cnt, pass_cnt, test_cnt - pass_cnt);
	printf("========================================= \r\n");

	vTaskDelete(NULL);
}

void example_sqlite(void)
{
	/*user can start their own task here*/
	if (xTaskCreate(example_sqlite_thread, ((const char *)"example_sqlite_thread"), 4096, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\r\n example_sqlite_thread: Create Task Error\n");
	}
}
