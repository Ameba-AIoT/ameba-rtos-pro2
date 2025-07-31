#include "cmsis.h"
#include "platform_stdlib.h"

#include "mbedtls/config.h"
#include "mbedtls/platform.h"
#include "mbedtls/ssl.h"
#include "mbedtls/debug.h"

__weak const char *client_cert_s = \
	"-----BEGIN CERTIFICATE-----\r\n" \
	"MIICHzCCAcagAwIBAgIBFDAKBggqhkjOPQQDAjBZMQswCQYDVQQGEwJBVTETMBEG\r\n" \
	"A1UECAwKU29tZS1TdGF0ZTEhMB8GA1UECgwYSW50ZXJuZXQgV2lkZ2l0cyBQdHkg\r\n" \
	"THRkMRIwEAYDVQQDDAlFQ0RTQV9DQTYwHhcNMjAwMzI0MDM0MTMxWhcNMzAwMzIy\r\n" \
	"MDM0MTMxWjBdMQswCQYDVQQGEwJBVTETMBEGA1UECAwKU29tZS1TdGF0ZTEhMB8G\r\n" \
	"A1UECgwYSW50ZXJuZXQgV2lkZ2l0cyBQdHkgTHRkMRYwFAYDVQQDDA1FQ0RTQV9D\r\n" \
	"TElFTlQ2MFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAEx6qNctKvddMx/oUV3VNj\r\n" \
	"sTS8yxFcjgDhfgOVRDA6iIlIMogjQPwQegWR/Snx2LD/0/xf0mTHQ7BJdoEENrQL\r\n" \
	"u6N7MHkwCQYDVR0TBAIwADAsBglghkgBhvhCAQ0EHxYdT3BlblNTTCBHZW5lcmF0\r\n" \
	"ZWQgQ2VydGlmaWNhdGUwHQYDVR0OBBYEFFFoG/T3NDvnylznv6N2WG28SgN5MB8G\r\n" \
	"A1UdIwQYMBaAFGpl48erOgKdfgTFI2vXI3Hl52NCMAoGCCqGSM49BAMCA0cAMEQC\r\n" \
	"IHVYmXzP1DYPIHQ5aGFICD5Ll3xtE6+HKVf1vEYCqiYGAiAluG4xvQce0Gu//YSn\r\n" \
	"TJcG/S+wkJm5xM86uIVT/xLA3Q==\r\n" \
	"-----END CERTIFICATE-----\r\n";

__weak const char *ca_cert_s = \
	"-----BEGIN CERTIFICATE-----\r\n" \
	"MIIB+TCCAZ+gAwIBAgIJAL74+aAry1YdMAoGCCqGSM49BAMCMFkxCzAJBgNVBAYT\r\n" \
	"AkFVMRMwEQYDVQQIDApTb21lLVN0YXRlMSEwHwYDVQQKDBhJbnRlcm5ldCBXaWRn\r\n" \
	"aXRzIFB0eSBMdGQxEjAQBgNVBAMMCUVDRFNBX0NBNjAeFw0yMDAzMjQwMzI4MjZa\r\n" \
	"Fw0zMDAzMjIwMzI4MjZaMFkxCzAJBgNVBAYTAkFVMRMwEQYDVQQIDApTb21lLVN0\r\n" \
	"YXRlMSEwHwYDVQQKDBhJbnRlcm5ldCBXaWRnaXRzIFB0eSBMdGQxEjAQBgNVBAMM\r\n" \
	"CUVDRFNBX0NBNjBZMBMGByqGSM49AgEGCCqGSM49AwEHA0IABOQUYRQ3dRPVnZ8l\r\n" \
	"GI5LR1z02x1Xz3R7Hh9qneYAdOKi1nShmr3UlqcM08w+byqKyhCVSRcFl+JoAzcK\r\n" \
	"iBM43v+jUDBOMB0GA1UdDgQWBBRqZePHqzoCnX4ExSNr1yNx5edjQjAfBgNVHSME\r\n" \
	"GDAWgBRqZePHqzoCnX4ExSNr1yNx5edjQjAMBgNVHRMEBTADAQH/MAoGCCqGSM49\r\n" \
	"BAMCA0gAMEUCIQCi1ONKJnIgdM0Y43+rXUk41VKXnL/jogdrocWWBVAtQwIgSYr2\r\n" \
	"tF3QdR4mHvyDXxGuyFYnjWjd1dSDyryGWTpBMDI=\r\n" \
	"-----END CERTIFICATE-----\r\n";

static void _debug(void *ctx, int level, const char *file, int line, const char *str)
{
	/* To avoid gcc warnings */
	( void ) ctx;
	( void ) level;
	
	dbg_printf("\n\r%s:%d: %s\n\r", file, line, str);
}

static int _verify(void *data, mbedtls_x509_crt *crt, int depth, uint32_t *flags) 
{
	char buf[1024];
	((void) data);

	dbg_printf("Verify requested for (Depth %d):\n", depth);
	mbedtls_x509_crt_info(buf, sizeof(buf) - 1, "", crt);
	dbg_printf("%s", buf);

	if(((*flags) & MBEDTLS_X509_BADCERT_EXPIRED) != 0)
		dbg_printf("server certificate has expired\n");

	if(((*flags) & MBEDTLS_X509_BADCERT_REVOKED) != 0)
		dbg_printf("  ! server certificate has been revoked\n");

	if(((*flags) & MBEDTLS_X509_BADCERT_CN_MISMATCH) != 0)
		dbg_printf("  ! CN mismatch\n");

	if(((*flags) & MBEDTLS_X509_BADCERT_NOT_TRUSTED) != 0)
		dbg_printf("  ! self-signed or not signed by a trusted CA\n");

	if(((*flags) & MBEDTLS_X509_BADCRL_NOT_TRUSTED) != 0)
		dbg_printf("  ! CRL not trusted\n");

	if(((*flags) & MBEDTLS_X509_BADCRL_EXPIRED) != 0)
		dbg_printf("  ! CRL expired\n");

	if(((*flags) & MBEDTLS_X509_BADCERT_OTHER) != 0)
		dbg_printf("  ! other (unknown) flag\n");

	if((*flags) == 0)
		dbg_printf("  Certificate verified without error flags\n");

	return(0);
}

void* __attribute__((cmse_nonsecure_call)) (*ns_calloc)(size_t, size_t) = NULL;
void __attribute__((cmse_nonsecure_call)) (*ns_free)(void *) = NULL;

int NS_ENTRY secure_mbedtls_platform_set_ns_calloc_free(
	void* (*calloc_func)(size_t, size_t),
	void (*free_func)(void *))
{
	ns_calloc = cmse_nsfptr_create((void* __attribute__((cmse_nonsecure_call)) (*)(size_t, size_t)) calloc_func);
	ns_free = cmse_nsfptr_create((void __attribute__((cmse_nonsecure_call)) (*)(void *)) free_func);

	return( 0 );
}

void *ns_calloc_call(size_t nmemb, size_t size)
{
	return ns_calloc(nmemb, size);
}

void ns_free_call(void *ptr)
{
	ns_free(ptr);
}

void NS_ENTRY secure_mbedtls_ssl_init(mbedtls_ssl_context *ssl)
{
	mbedtls_ssl_init(ssl);
}

void NS_ENTRY secure_mbedtls_ssl_conf_dbg(mbedtls_ssl_config *conf, void  *p_dbg)
{
#if defined(MBEDTLS_DEBUG_C)
	mbedtls_debug_set_threshold(0);
#endif
	mbedtls_ssl_conf_dbg(conf, _debug, p_dbg);
}

void NS_ENTRY secure_mbedtls_ssl_conf_verify(mbedtls_ssl_config *conf, void *p_vrfy)
{
	mbedtls_ssl_conf_verify(conf, _verify, p_vrfy);
}

int NS_ENTRY secure_mbedtls_ssl_conf_own_cert(
	mbedtls_ssl_config *conf,
	mbedtls_x509_crt *own_cert,
	mbedtls_pk_context *pk_key)
{
	return mbedtls_ssl_conf_own_cert(conf, own_cert, pk_key);
}

mbedtls_x509_crt* NS_ENTRY secure_mbedtls_x509_crt_parse(void)
{
	mbedtls_x509_crt *client_crt = (mbedtls_x509_crt *) mbedtls_calloc(1, sizeof(mbedtls_x509_crt));

	if(client_crt) {
		mbedtls_x509_crt_init(client_crt);

		if(mbedtls_x509_crt_parse(client_crt, client_cert_s, strlen((char const*) client_cert_s) + 1) != 0) {
			dbg_printf("\n\r ERROR: mbedtls_x509_crt_parse \n\r");
			goto error;
		}
	}
	else {
		dbg_printf("\n\r ERROR: mbedtls_calloc \n\r");
		goto error;
	}

	return client_crt;

error:
	if(client_crt) {
		mbedtls_x509_crt_free(client_crt);
		mbedtls_free(client_crt);
	}

	return NULL;
}

mbedtls_x509_crt* NS_ENTRY secure_mbedtls_x509_crt_parse_ca(void)
{
	mbedtls_x509_crt *ca_crt = (mbedtls_x509_crt *) mbedtls_calloc(1, sizeof(mbedtls_x509_crt));

	if(ca_crt) {
		mbedtls_x509_crt_init(ca_crt);

		if(mbedtls_x509_crt_parse(ca_crt, ca_cert_s, strlen((char const*) ca_cert_s) + 1) != 0) {
			dbg_printf("\n\r ERROR: mbedtls_x509_crt_parse \n\r");
			goto error;
		}
	}
	else {
		dbg_printf("\n\r ERROR: mbedtls_calloc \n\r");
		goto error;
	}

	return ca_crt;

error:
	if(ca_crt) {
		mbedtls_x509_crt_free(ca_crt);
		mbedtls_free(ca_crt);
	}

	return NULL;
}

void NS_ENTRY secure_mbedtls_x509_crt_free(mbedtls_x509_crt *crt)
{
	mbedtls_x509_crt_free(crt);
	mbedtls_free(crt);
}

int NS_ENTRY secure_mbedtls_ssl_setup(mbedtls_ssl_context *ssl, const mbedtls_ssl_config *conf)
{
	return mbedtls_ssl_setup(ssl, conf);
}

void NS_ENTRY secure_mbedtls_ssl_free(mbedtls_ssl_context *ssl)
{
	mbedtls_ssl_free(ssl);
}

void NS_ENTRY secure_mbedtls_ssl_config_free(mbedtls_ssl_config *conf)
{
	mbedtls_ssl_config_free(conf);
}

mbedtls_ssl_recv_t __attribute__((cmse_nonsecure_call)) *ns_f_recv = NULL;
mbedtls_ssl_recv_timeout_t __attribute__((cmse_nonsecure_call)) *ns_f_recv_timeout = NULL;
mbedtls_ssl_send_t __attribute__((cmse_nonsecure_call)) *ns_f_send = NULL;

int NS_ENTRY secure_mbedtls_ssl_handshake(mbedtls_ssl_context *ssl)
{
	ns_f_recv = cmse_nsfptr_create((mbedtls_ssl_recv_t __attribute__((cmse_nonsecure_call)) *) ssl->f_recv);
	ns_f_recv_timeout = cmse_nsfptr_create((mbedtls_ssl_recv_timeout_t __attribute__((cmse_nonsecure_call)) *) ssl->f_recv_timeout);
	ns_f_send = cmse_nsfptr_create((mbedtls_ssl_send_t __attribute__((cmse_nonsecure_call)) *) ssl->f_send);

	return mbedtls_ssl_handshake(ssl);
}

int ns_f_recv_call(void *ctx, unsigned char *buf, size_t len)
{
	return ns_f_recv(ctx, buf, len);
}

int ns_f_recv_timeout_call(void *ctx, unsigned char *buf, size_t len, uint32_t timeout)
{
	return ns_f_recv_timeout(ctx, buf, len, timeout);
}

int ns_f_send_call(void *ctx, const unsigned char *buf, size_t len)
{
	return ns_f_send(ctx, buf, len);
}

char* NS_ENTRY secure_mbedtls_ssl_get_ciphersuite(const mbedtls_ssl_context *ssl, char *buf)
{
	strcpy(buf, mbedtls_ssl_get_ciphersuite(ssl));
	return buf;
}

int NS_ENTRY secure_mbedtls_ssl_close_notify(mbedtls_ssl_context *ssl)
{
	return mbedtls_ssl_close_notify(ssl);
}

int NS_ENTRY secure_mbedtls_ssl_read(mbedtls_ssl_context *ssl, unsigned char *buf, size_t len)
{
	return mbedtls_ssl_read(ssl, buf, len);
}

int NS_ENTRY secure_mbedtls_ssl_write(mbedtls_ssl_context *ssl, const unsigned char *buf, size_t len)
{
	return mbedtls_ssl_write(ssl, buf, len);
}
