#include <cmsis.h>
#include <platform_stdlib.h>
#if defined(CONFIG_PLATFORM_8710C)
#include <hal_efuse.h>
#define USER_PARTITION_OFFSET 0x210000
#elif defined(CONFIG_PLATFORM_8735B)
#include <efuse_api.h>
#include <platform_opts.h>
#define USER_PARTITION_OFFSET SECURE_STORAGE_BASS
#endif
#include <crypto_api.h>
#include <device_lock.h>

#define secure_storage_debug  printf

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

static const unsigned char aes_gcm_iv[12] __attribute__((aligned(32))) = {
	0xee, 0xc3, 0x13, 0xdd, 0x07, 0xcc, 0x1b, 0x3e, 0x6b, 0x06, 0x8a, 0x47
};

static const unsigned char aes_gcm_aad[20] __attribute__((aligned(32))) = {
	0x13, 0x89, 0xb5, 0x22,
	0xc2, 0x4a, 0x77, 0x41,
	0x81, 0x70, 0x05, 0x53,
	0xf0, 0x24, 0x6b, 0xba,
	0xbd, 0xd3, 0x8d, 0x6f
};

#if defined(__ICCARM__)
static void (__cmse_nonsecure_call *ns_flash_erase)(uint32_t) = NULL;
static int (__cmse_nonsecure_call *ns_flash_read)(uint32_t, uint32_t, uint8_t *) = NULL;
static int (__cmse_nonsecure_call *ns_flash_write)(uint32_t, uint32_t, uint8_t *) = NULL;
static void (__cmse_nonsecure_call *ns_device_lock)(uint32_t) = NULL;
static void (__cmse_nonsecure_call *ns_device_unlock)(uint32_t) = NULL;
#else
typedef void __attribute__((cmse_nonsecure_call))(*ns_flash_erase_t)(uint32_t);
typedef int __attribute__((cmse_nonsecure_call))(*ns_flash_read_t)(uint32_t, uint32_t, uint8_t *);
typedef int __attribute__((cmse_nonsecure_call))(*ns_flash_write_t)(uint32_t, uint32_t, uint8_t *);
typedef void __attribute__((cmse_nonsecure_call))(*ns_device_lock_t)(uint32_t);
typedef void __attribute__((cmse_nonsecure_call))(*ns_device_unlock_t)(uint32_t);

static ns_flash_erase_t ns_flash_erase = NULL;
static ns_flash_read_t ns_flash_read = NULL;
static ns_flash_write_t ns_flash_write = NULL;
static ns_device_lock_t ns_device_lock = NULL;
static ns_device_unlock_t ns_device_unlock = NULL;
#endif
static uint8_t *ns_buf = NULL;
static uint8_t crypto_inited = 0;
static user_data_t g_user_data = {0};

static int secure_storage_encrypt_data(uint8_t *input, uint32_t input_len, uint8_t *output, uint8_t *tag)
{
	int ret = 0;
	uint8_t tmpbuf[64];
	uint8_t *key = (uint8_t *)(((uint32_t) tmpbuf + 31) & (~31)); // align to 32byte

	if (!crypto_inited) {
		ns_device_lock(RT_DEV_LOCK_CRYPTO);
		ret = crypto_init();
		ns_device_unlock(RT_DEV_LOCK_CRYPTO);
		if (ret == 0) {
			crypto_inited = 1;
		} else {
			secure_storage_debug("\n\r ERROR: crypto_init \n\r");
			goto exit;
		}
	}

	memset(key, 0, 32);
	ns_device_lock(RT_DEV_LOCK_EFUSE);
#if defined(CONFIG_PLATFORM_8710C)
	/* get from secure efuse 2nd key */
	/* 1st key is for secure boot hash */
	hal_sec_key_get(key, 1, 32);
#elif defined(CONFIG_PLATFORM_8735B)
	efuse_crypto_key_get(key, 0);
#endif
	ns_device_unlock(RT_DEV_LOCK_EFUSE);

	ns_device_lock(RT_DEV_LOCK_CRYPTO);
	ret = crypto_aes_gcm_init(key, 32);
	if (ret != 0) {
		ns_device_unlock(RT_DEV_LOCK_CRYPTO);
		secure_storage_debug("\n\r ERROR: crypto_aes_gcm_init \n\r");
		goto exit;
	}

	ret = crypto_aes_gcm_encrypt(input, input_len, aes_gcm_iv, aes_gcm_aad, sizeof(aes_gcm_aad), output, tag);
	ns_device_unlock(RT_DEV_LOCK_CRYPTO);
	if (ret != 0) {
		secure_storage_debug("\n\r ERROR: crypto_aes_gcm_encrypt \n\r");
		goto exit;
	}

exit:
	return ret;
}

static int secure_storage_decrypt_data(uint8_t *input, uint32_t input_len, uint8_t *output, uint8_t *tag)
{
	int ret = 0;
	uint8_t dec_tag[16];
	uint8_t tmpbuf[64];
	uint8_t *key = (uint8_t *)(((uint32_t) tmpbuf + 31) & (~31)); // align to 32byte

	if (!crypto_inited) {
		ns_device_lock(RT_DEV_LOCK_CRYPTO);
		ret = crypto_init();
		ns_device_unlock(RT_DEV_LOCK_CRYPTO);
		if (ret == 0) {
			crypto_inited = 1;
		} else {
			secure_storage_debug("\n\r ERROR: crypto_init \n\r");
			goto exit;
		}
	}

	memset(key, 0, 32);
	ns_device_lock(RT_DEV_LOCK_EFUSE);
#if defined(CONFIG_PLATFORM_8710C)
	/* get from secure efuse 2nd key */
	/* 1st key is for secure boot hash */
	hal_sec_key_get(key, 1, 32);
#elif defined(CONFIG_PLATFORM_8735B)
	efuse_crypto_key_get(key, 0);
#endif
	ns_device_unlock(RT_DEV_LOCK_EFUSE);

	ns_device_lock(RT_DEV_LOCK_CRYPTO);
	ret = crypto_aes_gcm_init(key, 32);
	if (ret != 0) {
		ns_device_unlock(RT_DEV_LOCK_CRYPTO);
		secure_storage_debug("\n\r ERROR: crypto_aes_gcm_init \n\r");
		goto exit;
	}

	ret = crypto_aes_gcm_decrypt(input, input_len, aes_gcm_iv, aes_gcm_aad, sizeof(aes_gcm_aad), output, dec_tag);
	ns_device_unlock(RT_DEV_LOCK_CRYPTO);
	if (ret != 0) {
		secure_storage_debug("\n\r ERROR: crypto_aes_gcm_decrypt \n\r");
		goto exit;
	}

	/* verify tag */
	if (memcmp(dec_tag, tag, 16) != 0) {
		secure_storage_debug("\n\r ERROR: memcmp tag \n\r");
		ret = -1;
	}

exit:
	return ret;
}

int NS_ENTRY secure_storage_set_ns_func(ns_func_t *func, uint8_t *buf, uint32_t buf_len)
{
	int ret = 0;

#if defined(__ICCARM__)
	ns_flash_erase = (void (__cmse_nonsecure_call *)(uint32_t)) func->flash_erase;
	ns_flash_read = (int (__cmse_nonsecure_call *)(uint32_t, uint32_t, uint8_t *)) func->flash_read;
	ns_flash_write = (int (__cmse_nonsecure_call *)(uint32_t, uint32_t, uint8_t *)) func->flash_write;
	ns_device_lock = (void (__cmse_nonsecure_call *)(uint32_t)) func->device_lock;
	ns_device_unlock = (void (__cmse_nonsecure_call *)(uint32_t)) func->device_unlock;
#else
	ns_flash_erase = (ns_flash_erase_t)cmse_nsfptr_create(func->flash_erase);
	ns_flash_read = (ns_flash_read_t)cmse_nsfptr_create(func->flash_read);
	ns_flash_write = (ns_flash_write_t)cmse_nsfptr_create(func->flash_write);
	ns_device_lock = (ns_device_lock_t)cmse_nsfptr_create(func->device_lock);
	ns_device_unlock = (ns_device_unlock_t)cmse_nsfptr_create(func->device_unlock);
#endif
	ns_buf = buf;

	if (buf_len < ((sizeof(user_data_t) + 15) / 16 * 16 + 16)) { // AES blocks + GCM TAG
		ret = -1;
	}

	return ret;
}

void NS_ENTRY secure_storage_setup_key(uint8_t *key)
{
#if defined(CONFIG_PLATFORM_8710C)
	hal_sec_key_write(key, 1);
#elif defined(CONFIG_PLATFORM_8735B)
	efuse_crypto_key_write(key, 0, 1);
#endif
}

int NS_ENTRY secure_storage_check_empty(void)
{
	int is_empty = 1;

	ns_flash_read(USER_PARTITION_OFFSET, sizeof(user_data_t), ns_buf);

	for (int i = 0; i < sizeof(user_data_t); i ++) {
		if (ns_buf[i] != 0xff) {
			is_empty = 0;
			break;
		}
	}

	secure_storage_debug("\n\r %s %d \n\r", __func__, is_empty);
	return is_empty;
}

void NS_ENTRY secure_storage_setup_user_data(user_data_t *user_data)
{
	uint32_t data_len = (sizeof(user_data_t) + 15) / 16 * 16; // AES blocks
	uint8_t *tag = ns_buf + data_len;
	uint8_t *user_data_plain = (uint8_t *) malloc(data_len);

	if (user_data_plain) {
		memset(user_data_plain, 0, data_len);
		memcpy(user_data_plain, user_data, sizeof(user_data_t));
		secure_storage_encrypt_data(user_data_plain, data_len, ns_buf, tag);
		ns_flash_erase(USER_PARTITION_OFFSET);
		ns_flash_write(USER_PARTITION_OFFSET, data_len + 16, ns_buf);	// AES blocks + GCM tag
		free(user_data_plain);
	}
}

void NS_ENTRY secure_storage_load_user_data(void)
{
	uint32_t data_len = (sizeof(user_data_t) + 15) / 16 * 16; // AES blocks
	uint8_t *tag = ns_buf + data_len;
	uint8_t *user_data_plain = (uint8_t *) malloc(data_len);

	if (user_data_plain) {
		ns_flash_read(USER_PARTITION_OFFSET, data_len + 16, ns_buf); // AES blocks + GCM TAG

		if (secure_storage_decrypt_data(ns_buf, data_len, user_data_plain, tag) == 0) {
			memcpy(&g_user_data, user_data_plain, sizeof(user_data_t));

			secure_storage_debug(" %s client_key:\n\r%s\n\r", __func__, g_user_data.client_key);
		}

		free(user_data_plain);
	}
}
