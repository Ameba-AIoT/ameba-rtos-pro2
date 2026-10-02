#include "cmsis.h"
#include "platform_stdlib.h"

#include "mbedtls/config.h"
#include "mbedtls/platform.h"
#include "mbedtls/ssl.h"
#include "crypto_api.h"

__weak const char *client_key_s = \
	"-----BEGIN EC PARAMETERS-----\r\n" \
	"BggqhkjOPQMBBw==\r\n" \
	"-----END EC PARAMETERS-----\r\n" \
	"-----BEGIN EC PRIVATE KEY-----\r\n" \
	"MHcCAQEEIMCUIrYr3bTfxGhOdDZnHoyxZa9lwH3dnkGSPPUVHFn8oAoGCCqGSM49\r\n" \
	"AwEHoUQDQgAEx6qNctKvddMx/oUV3VNjsTS8yxFcjgDhfgOVRDA6iIlIMogjQPwQ\r\n" \
	"egWR/Snx2LD/0/xf0mTHQ7BJdoEENrQLuw==\r\n" \
	"-----END EC PRIVATE KEY-----\r\n";

static void* _calloc(size_t count, size_t size)
{
	void *ptr = pvPortMalloc(count * size);
	if(ptr)	memset(ptr, 0, count * size);
	return ptr;
}

#define _free		vPortFree

void __attribute__((cmse_nonsecure_call)) (*ns_device_mutex_lock)(uint32_t) = NULL;
void __attribute__((cmse_nonsecure_call)) (*ns_device_mutex_unlock)(uint32_t) = NULL;

void NS_ENTRY secure_set_ns_device_lock(
	void (*device_mutex_lock_func)(uint32_t),
	void (*device_mutex_unlock_func)(uint32_t))
{
	ns_device_mutex_lock = cmse_nsfptr_create((void __attribute__((cmse_nonsecure_call)) (*)(uint32_t)) device_mutex_lock_func);
	ns_device_mutex_unlock = cmse_nsfptr_create((void __attribute__((cmse_nonsecure_call)) (*)(uint32_t)) device_mutex_unlock_func);
}

void ns_device_mutex_lock_call(uint32_t device)
{
	ns_device_mutex_lock(device);
}

void ns_device_mutex_unlock_call(uint32_t device)
{
	ns_device_mutex_unlock(device);
}

int NS_ENTRY secure_mbedtls_platform_set_calloc_free(void)
{
	return 	mbedtls_platform_set_calloc_free(_calloc, _free);
}

psa_status_t NS_ENTRY secure_psa_crypto_init(void)
{
	return psa_crypto_init();
}

mbedtls_pk_context* NS_ENTRY secure_mbedtls_pk_parse_key(void)
{

	mbedtls_pk_context *client_pk = (mbedtls_pk_context *) mbedtls_calloc(1, sizeof(mbedtls_pk_context));

	if(client_pk) {
		mbedtls_pk_init(client_pk);

		if(mbedtls_pk_parse_key(client_pk, (unsigned char const *)client_key_s, strlen(client_key_s) + 1, NULL, 0) != 0) {
			dbg_printf("\n\r ERROR: mbedtls_pk_parse_key \n\r");
			goto error;
		}
	}
	else {
		dbg_printf("\n\r ERROR: mbedtls_calloc \n\r");
		goto error;
	}

	return client_pk;

error:
	if(client_pk) {
		mbedtls_pk_free(client_pk);
		mbedtls_free(client_pk);
	}

	return NULL;
}

void NS_ENTRY secure_mbedtls_pk_free(mbedtls_pk_context *pk)
{
	mbedtls_pk_free(pk);
	mbedtls_free(pk);
}

unsigned char NS_ENTRY secure_mbedtls_ssl_sig_from_pk(mbedtls_pk_context *pk)
{
	return mbedtls_ssl_sig_from_pk(pk);
}

struct secure_mbedtls_pk_sign_param {
	mbedtls_pk_context *ctx;
	mbedtls_md_type_t md_alg;
	unsigned char *hash;
	size_t hash_len;
	unsigned char *sig;
	size_t sig_size;
	size_t *sig_len;
};

int NS_ENTRY secure_mbedtls_pk_sign(struct secure_mbedtls_pk_sign_param *param)
{
	return mbedtls_pk_sign(param->ctx, param->md_alg, param->hash, param->hash_len,
			param->sig, param->sig_size, param->sig_len);
}

#if defined(MBEDTLS_HAVE_TIME) && defined(MBEDTLS_PLATFORM_MS_TIME_ALT)
mbedtls_ms_time_t __attribute__((cmse_nonsecure_call)) (*ns_ms_time)(void) = NULL;

void NS_ENTRY secure_set_ns_ms_time(mbedtls_ms_time_t (*ms_time_func)(void))
{
	ns_ms_time = cmse_nsfptr_create((void __attribute__((cmse_nonsecure_call)) (*)(uint32_t)) ms_time_func);
}

mbedtls_ms_time_t ns_ms_time_call(void)
{
	if (ns_ms_time) {
		return ns_ms_time();
	} else {
		return 0;
	}
}
#endif
