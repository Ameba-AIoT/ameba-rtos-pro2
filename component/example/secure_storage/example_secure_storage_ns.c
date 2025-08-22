#include <cmsis.h>
#include <osdep_service.h>
#if defined(CONFIG_PLATFORM_8710C)
#include <flash_api.h>
#elif defined(CONFIG_PLATFORM_8735B)
#include <ftl_common_api.h>
#endif
#include <device_lock.h>

#define STACKSIZE     2048
#define NS_BUF_LEN    4096

typedef struct user_data_s {
	char client_key[512];
} user_data_t;

typedef struct ns_func_s {
	void (*flash_erase)(uint32_t);
	int (*flash_read)(uint32_t, uint32_t, uint8_t *);
	int (*flash_write)(uint32_t, uint32_t, uint8_t *);
	void (*device_lock)(uint32_t);
	void (*device_unlock)(uint32_t);
} ns_func_t;

extern void NS_ENTRY secure_storage_setup_key(uint8_t *key);
extern int NS_ENTRY secure_storage_set_ns_func(ns_func_t *func, uint8_t *buf, uint32_t buf_len);
extern int NS_ENTRY secure_storage_check_empty(void);
extern void NS_ENTRY secure_storage_setup_user_data(user_data_t *user_data);
extern void NS_ENTRY secure_storage_load_user_data(void);

static void ns_flash_erase(uint32_t address)
{
	flash_t flash;

#if defined(CONFIG_PLATFORM_8710C)
	device_mutex_lock(RT_DEV_LOCK_FLASH);
	flash_erase_sector(&flash, address);
	device_mutex_unlock(RT_DEV_LOCK_FLASH);
#elif defined(CONFIG_PLATFORM_8735B)
	ftl_erase_sector(address);
#endif
}

static int ns_flash_read(uint32_t address, uint32_t len, uint8_t *data)
{
	int ret;
	flash_t flash;

#if defined(CONFIG_PLATFORM_8710C)
	device_mutex_lock(RT_DEV_LOCK_FLASH);
	ret = flash_stream_read(&flash, address, len, data);
	device_mutex_unlock(RT_DEV_LOCK_FLASH);
#elif defined(CONFIG_PLATFORM_8735B)
	ret = ftl_common_read(address, data, len);
#endif

	return ret;
}

static int ns_flash_write(uint32_t address, uint32_t len, uint8_t *data)
{
	int ret;
	flash_t flash;

#if defined(CONFIG_PLATFORM_8710C)
	device_mutex_lock(RT_DEV_LOCK_FLASH);
	ret = flash_stream_write(&flash, address, len, data);
	device_mutex_unlock(RT_DEV_LOCK_FLASH);
#elif defined(CONFIG_PLATFORM_8735B)
	ret = ftl_common_write(address, data, len);
#endif

	return ret;
}

static void example_secure_storage_thread(void *param)
{
	/* To avoid gcc warnings */
	(void) param;

	rtw_create_secure_context(STACKSIZE * 4);

	/* NOTE: efuse sec key bytes can only be written once !!!
	   If key is not setup, 0xFF is the default value in efuse.
	   May change the following key value written to efuse */
#if 0
	uint8_t key[32] = {
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	};
	secure_storage_setup_key(key);
#endif

	uint8_t *ns_buf = (uint8_t *) malloc(NS_BUF_LEN);
	if (ns_buf == NULL) {
		goto exit;
	}

	ns_func_t func = {ns_flash_erase, ns_flash_read, ns_flash_write, device_mutex_lock, device_mutex_unlock};
	if (secure_storage_set_ns_func(&func, ns_buf, NS_BUF_LEN) != 0) {
		goto exit;
	}

	if (secure_storage_check_empty()) {
		user_data_t *user_data = (user_data_t *) malloc(sizeof(user_data_t));

		if (user_data) {
			memset(user_data, 0, sizeof(user_data_t));
			const char *client_key = \
									 "-----BEGIN EC PARAMETERS-----\r\n" \
									 "BggqhkjOPQMBBw==\r\n" \
									 "-----END EC PARAMETERS-----\r\n" \
									 "-----BEGIN EC PRIVATE KEY-----\r\n" \
									 "MHcCAQEEIAQxciQmaeuPLUa8VueFj9fTEdqbqLY8jW84NuKtxf+ToAoGCCqGSM49\r\n" \
									 "AwEHoUQDQgAEtxzt0vQIGEeVZMklv+ZJnkjpSj9IlhZhRyfY4rFyieaD3wo3cnw0\r\n" \
									 "LJTKAEZCGC7y+4qzkzFY/FVUG+zxwkWVsw==\r\n" \
									 "-----END EC PRIVATE KEY-----\r\n";
			strcpy(user_data->client_key, client_key);
			secure_storage_setup_user_data(user_data);
			free(user_data);
		}
	}

	secure_storage_load_user_data();

exit:
	if (ns_buf) {
		free(ns_buf);
	}

	vTaskDelete(NULL);
}

void example_secure_storage(void)
{
	if (xTaskCreate(example_secure_storage_thread, ((const char *)"example_secure_storage_thread"), STACKSIZE, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(example_secure_storage_thread) failed", __FUNCTION__);
	}
}
