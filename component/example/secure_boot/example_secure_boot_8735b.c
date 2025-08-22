#include "FreeRTOS.h"
#include "task.h"
#include <platform_stdlib.h>
#include "osdep_service.h"
#include "device_lock.h"
#include "hal_otp.h"

#define ROTPK_HASH_LEN  32
#define HUK_LEN         32
#define SEC_KEY_LEN     32

#define BUF_LEN         32

extern int otp_image_hash_enable(void);
extern int otp_trust_boot_enable(void);
extern int otp_secure_boot_enable(void);
extern int otp_boot_verify_enabled(void);

// key1_pkhash in GCC-RELEASE\build\key_private.json
const uint8_t rotpk_hash[ROTPK_HASH_LEN] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

// HUK key1 in GCC-RELEASE\build\key_private.json
const uint8_t huk[HUK_LEN] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

// SEC key1 in GCC-RELEASE\build\key_private.json
const uint8_t sec_key[SEC_KEY_LEN] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

static void example_secure_boot_thread(void *param)
{
	int ret = 0;
	u8 read_buf[BUF_LEN];

#if defined(configENABLE_TRUSTZONE) && (configENABLE_TRUSTZONE == 1)
	rtw_create_secure_context(configMINIMAL_SECURE_STACK_SIZE);
#endif

	printf("\r\nExample: secure_boot\r\n");
	hal_otp_init();

	/*
	 * Program the ROTPK SHA256 hash if image hash check or trust boot or secure boot are enabled
	 */
	// read ROTPK hash
	memset(read_buf, 0, BUF_LEN);
	device_mutex_lock(RT_DEV_LOCK_EFUSE);
	ret = hal_otp_sb_key_get(read_buf, SB_OTP_HIGH_VAL_ROTPK_HSH1);
	device_mutex_unlock(RT_DEV_LOCK_EFUSE);
	if (ret < 0) {
		printf("ERROR: hal_otp_sb_key_get ROTPK hash\r\n");
		goto exit;
	}
	for (int i = 0; i < ROTPK_HASH_LEN; i += 8) {
		printf("[%d]\t%02X %02X %02X %02X  %02X %02X %02X %02X\r\n",
			   i, read_buf[i], read_buf[i + 1], read_buf[i + 2], read_buf[i + 3], read_buf[i + 4], read_buf[i + 5], read_buf[i + 6], read_buf[i + 7]);
	}
	// write ROTPK hash
	if (0) {
		device_mutex_lock(RT_DEV_LOCK_EFUSE);
		ret = hal_otp_sb_key_write(rotpk_hash, SB_OTP_HIGH_VAL_ROTPK_HSH1);
		device_mutex_unlock(RT_DEV_LOCK_EFUSE);
		if (ret < 0) {
			printf("ERROR: hal_otp_sb_key_write ROTPK hash\r\n");
			goto exit;
		}
		printf("\r\nWrite ROTPK Hash Done.\r\n");

		// read ROTPK hash
		memset(read_buf, 0, BUF_LEN);
		device_mutex_lock(RT_DEV_LOCK_EFUSE);
		ret = hal_otp_sb_key_get(read_buf, SB_OTP_HIGH_VAL_ROTPK_HSH1);
		device_mutex_unlock(RT_DEV_LOCK_EFUSE);
		if (ret < 0) {
			printf("ERROR: hal_otp_sb_key_get ROTPK hash\r\n");
			goto exit;
		}
		for (int i = 0; i < ROTPK_HASH_LEN; i += 8) {
			printf("[%d]\t%02X %02X %02X %02X  %02X %02X %02X %02X\r\n",
				   i, read_buf[i], read_buf[i + 1], read_buf[i + 2], read_buf[i + 3], read_buf[i + 4], read_buf[i + 5], read_buf[i + 6], read_buf[i + 7]);
		}

		if (memcmp(read_buf, rotpk_hash, ROTPK_HASH_LEN)) {
			printf("ERROR: memcmp ROTPK hash\r\n");
			goto exit;
		}
	} else {
		printf("\r\nNot program ROTPK hash to efuse.\r\n");
	}

	/*
	 * Program the HUK on eFUSE if hmac-sha256 is used for image hash check
	 */
	// read HUK
	memset(read_buf, 0, BUF_LEN);
	device_mutex_lock(RT_DEV_LOCK_EFUSE);
	ret = hal_otp_sb_key_get(read_buf, SB_OTP_HIGH_VAL_HUK1);
	device_mutex_unlock(RT_DEV_LOCK_EFUSE);
	if (ret < 0) {
		printf("ERROR: hal_otp_sb_key_get HUK\r\n");
		goto exit;
	}
	for (int i = 0; i < HUK_LEN; i += 8) {
		printf("[%d]\t%02X %02X %02X %02X  %02X %02X %02X %02X\r\n",
			   i, read_buf[i], read_buf[i + 1], read_buf[i + 2], read_buf[i + 3], read_buf[i + 4], read_buf[i + 5], read_buf[i + 6], read_buf[i + 7]);
	}
	// write HUK
	if (0) {
		device_mutex_lock(RT_DEV_LOCK_EFUSE);
		ret = hal_otp_sb_key_write(huk, SB_OTP_HIGH_VAL_HUK1);
		device_mutex_unlock(RT_DEV_LOCK_EFUSE);
		if (ret < 0) {
			printf("ERROR: hal_otp_sb_key_write HUK\r\n");
			goto exit;
		}
		printf("\r\nWrite HUK Done.\r\n");

		// read HUK
		memset(read_buf, 0, BUF_LEN);
		device_mutex_lock(RT_DEV_LOCK_EFUSE);
		ret = hal_otp_sb_key_get(read_buf, SB_OTP_HIGH_VAL_HUK1);
		device_mutex_unlock(RT_DEV_LOCK_EFUSE);
		if (ret < 0) {
			printf("ERROR: hal_otp_sb_key_get HUK\r\n");
			goto exit;
		}
		for (int i = 0; i < HUK_LEN; i += 8) {
			printf("[%d]\t%02X %02X %02X %02X  %02X %02X %02X %02X\r\n",
				   i, read_buf[i], read_buf[i + 1], read_buf[i + 2], read_buf[i + 3], read_buf[i + 4], read_buf[i + 5], read_buf[i + 6], read_buf[i + 7]);
		}

		if (memcmp(read_buf, huk, HUK_LEN)) {
			printf("ERROR: memcmp HUK\r\n");
			goto exit;
		}
	} else {
		printf("\r\nNot program HUK to efuse.\r\n");
	}

	/*
	 * Program the SEC key if image payload decrypt is used
	 */
	// read SEC key
	memset(read_buf, 0, BUF_LEN);
	device_mutex_lock(RT_DEV_LOCK_EFUSE);
	ret = hal_otp_sb_key_get(read_buf, SB_OTP_HIGH_VAL_SEC_KEY1);
	device_mutex_unlock(RT_DEV_LOCK_EFUSE);
	if (ret < 0) {
		printf("ERROR: hal_otp_sb_key_get SEC key\r\n");
		goto exit;
	}
	for (int i = 0; i < SEC_KEY_LEN; i += 8) {
		printf("[%d]\t%02X %02X %02X %02X  %02X %02X %02X %02X\r\n",
			   i, read_buf[i], read_buf[i + 1], read_buf[i + 2], read_buf[i + 3], read_buf[i + 4], read_buf[i + 5], read_buf[i + 6], read_buf[i + 7]);
	}
	// write SEC key
	if (0) {
		device_mutex_lock(RT_DEV_LOCK_EFUSE);
		ret = hal_otp_sb_key_write(sec_key, SB_OTP_HIGH_VAL_SEC_KEY1);
		device_mutex_unlock(RT_DEV_LOCK_EFUSE);
		if (ret < 0) {
			printf("ERROR: hal_otp_sb_key_write SEC key\r\n");
			goto exit;
		}
		printf("\r\nWrite SEC Key Done.\r\n");

		// read SEC key
		memset(read_buf, 0, BUF_LEN);
		device_mutex_lock(RT_DEV_LOCK_EFUSE);
		ret = hal_otp_sb_key_get(read_buf, SB_OTP_HIGH_VAL_SEC_KEY1);
		device_mutex_unlock(RT_DEV_LOCK_EFUSE);
		if (ret < 0) {
			printf("ERROR: hal_otp_sb_key_get SEC key\r\n");
			goto exit;
		}
		for (int i = 0; i < SEC_KEY_LEN; i += 8) {
			printf("[%d]\t%02X %02X %02X %02X  %02X %02X %02X %02X\r\n",
				   i, read_buf[i], read_buf[i + 1], read_buf[i + 2], read_buf[i + 3], read_buf[i + 4], read_buf[i + 5], read_buf[i + 6], read_buf[i + 7]);
		}

		if (memcmp(read_buf, sec_key, SEC_KEY_LEN)) {
			printf("ERROR: memcmp SEC key\r\n");
			goto exit;
		}
	} else {
		printf("\r\nNot program SEC key to efuse.\r\n");
	}

	/*
	 * Lock super secure zone to protect the HUK and SEC key from being read by CPU
	 */
	// lock super secure zone, make HUK and SEC key unreadable forever.
	// this configure is irreversible, so please do this only if you are certain about keys
	if (0) {
		device_mutex_lock(RT_DEV_LOCK_EFUSE);
		ret = hal_otp_ssz_lock();
		device_mutex_unlock(RT_DEV_LOCK_EFUSE);
		if (ret < 0) {
			printf("ERROR: hal_otp_ssz_lock\r\n");
			goto exit;
		}
	}

	/*
	 * Enable the secure boot so that device will only boot with secure boot image
	 */
	// enable secure boot, make device boot only with correctly secure boot image
	// this configure is irreversible, so please do this only if you are certain that the image is generated with the correct keys
	if (0) {
		if (0) {
			// enable ROTPK hash check + key certificate signature check
			// enable image hash check + section load check
			// must program ROTPK hash to eFUSE
			// must program HUK to eFUSE if use hmac-sha256
			ret = otp_image_hash_enable();
		} else if (0) {
			// enable ROTPK hash check + key certificate signature check
			// enable manifest signature check + image hash check + section load check
			// must program ROTPK hash to eFUSE
			// must program HUK to eFUSE if use hmac-sha256
			ret = otp_trust_boot_enable();
		} else if (0) {
			// enable ROTPK hash check + key certificate signature check
			// enable manifest signature check + image hash check + image payload decrypt + section load check
			// must program ROTPK hash to eFUSE
			// must program HUK to eFUSE if use hmac-sha256
			// must program SEK key to eFUSE
			ret = otp_secure_boot_enable();
		}

		if (ret < 0) {
			printf("\r\nsecure boot enable error\r\n");
			goto exit;
		}

		ret = otp_boot_verify_enabled();
		if (ret) {
			printf("\r\nsecure boot is enabled\r\n");
		}
	}

exit:

	vTaskDelete(NULL);
}

void example_secure_boot(void)
{
	if (xTaskCreate(example_secure_boot_thread, ((const char *)"example_secure_boot_thread"), 2048, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
		printf("\n\r%s xTaskCreate(example_secure_boot_thread) failed", __FUNCTION__);
	}
}