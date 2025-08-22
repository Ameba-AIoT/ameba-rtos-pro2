#include "cmsis.h"
#include "rt_printf.h"
#include "diag.h"
#include "crypto_api.h"
#include "platform_stdlib.h"

#include "mbedtls/config.h"
#include "mbedtls/platform.h"
#include "mbedtls/bignum.h"
#include "mbedtls/ecp.h"

const char *client_key_s = \
						   "-----BEGIN EC PARAMETERS-----\r\n" \
						   "BggqhkjOPQMBBw==\r\n" \
						   "-----END EC PARAMETERS-----\r\n" \
						   "-----BEGIN EC PRIVATE KEY-----\r\n" \
						   "MHcCAQEEIAQxciQmaeuPLUa8VueFj9fTEdqbqLY8jW84NuKtxf+ToAoGCCqGSM49\r\n" \
						   "AwEHoUQDQgAEtxzt0vQIGEeVZMklv+ZJnkjpSj9IlhZhRyfY4rFyieaD3wo3cnw0\r\n" \
						   "LJTKAEZCGC7y+4qzkzFY/FVUG+zxwkWVsw==\r\n" \
						   "-----END EC PRIVATE KEY-----\r\n";

const unsigned char *client_cert_s = \
									 "-----BEGIN CERTIFICATE-----\r\n" \
									 "MIICIDCCAcagAwIBAgIBDzAKBggqhkjOPQQDAjBZMQswCQYDVQQGEwJBVTETMBEG\r\n" \
									 "A1UECAwKU29tZS1TdGF0ZTEhMB8GA1UECgwYSW50ZXJuZXQgV2lkZ2l0cyBQdHkg\r\n" \
									 "THRkMRIwEAYDVQQDDAlFQ0RTQV9DQTUwHhcNMTkwMzE4MDkwNzI5WhcNMjAwMzE3\r\n" \
									 "MDkwNzI5WjBdMQswCQYDVQQGEwJBVTETMBEGA1UECAwKU29tZS1TdGF0ZTEhMB8G\r\n" \
									 "A1UECgwYSW50ZXJuZXQgV2lkZ2l0cyBQdHkgTHRkMRYwFAYDVQQDDA1FQ0RTQV9D\r\n" \
									 "TElFTlQ1MFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAEtxzt0vQIGEeVZMklv+ZJ\r\n" \
									 "nkjpSj9IlhZhRyfY4rFyieaD3wo3cnw0LJTKAEZCGC7y+4qzkzFY/FVUG+zxwkWV\r\n" \
									 "s6N7MHkwCQYDVR0TBAIwADAsBglghkgBhvhCAQ0EHxYdT3BlblNTTCBHZW5lcmF0\r\n" \
									 "ZWQgQ2VydGlmaWNhdGUwHQYDVR0OBBYEFKOsaCZtravcnWFu2jPOgybGniWJMB8G\r\n" \
									 "A1UdIwQYMBaAFDuPlIvp73SZcqD8gvPLS9xwVei+MAoGCCqGSM49BAMCA0gAMEUC\r\n" \
									 "IQDmE+cRBvi7xRbr5x0xuzLfou01WX/bfXFKi77Hmc6OOQIgZSJaKiJ8QmmE53JF\r\n" \
									 "OIyY8szV/j7Rg2qHVPQhwCVJcp0=\r\n" \
									 "-----END CERTIFICATE-----\r\n";

const unsigned char *ca_cert_s = \
								 "-----BEGIN CERTIFICATE-----\r\n" \
								 "MIIB+TCCAZ+gAwIBAgIJAIRr3KcObsW7MAoGCCqGSM49BAMCMFkxCzAJBgNVBAYT\r\n" \
								 "AkFVMRMwEQYDVQQIDApTb21lLVN0YXRlMSEwHwYDVQQKDBhJbnRlcm5ldCBXaWRn\r\n" \
								 "aXRzIFB0eSBMdGQxEjAQBgNVBAMMCUVDRFNBX0NBNTAeFw0xOTAzMTgwOTAyNDBa\r\n" \
								 "Fw0yMDAzMTcwOTAyNDBaMFkxCzAJBgNVBAYTAkFVMRMwEQYDVQQIDApTb21lLVN0\r\n" \
								 "YXRlMSEwHwYDVQQKDBhJbnRlcm5ldCBXaWRnaXRzIFB0eSBMdGQxEjAQBgNVBAMM\r\n" \
								 "CUVDRFNBX0NBNTBZMBMGByqGSM49AgEGCCqGSM49AwEHA0IABGfODWzfkxx+ScTn\r\n" \
								 "0AL17wjEMcohhwzuaSOJFRoVwV3avuCHtfQcxcVJBslrRjcK/8xeo/prE/JaHyEK\r\n" \
								 "wYIhYuajUDBOMB0GA1UdDgQWBBQ7j5SL6e90mXKg/ILzy0vccFXovjAfBgNVHSME\r\n" \
								 "GDAWgBQ7j5SL6e90mXKg/ILzy0vccFXovjAMBgNVHRMEBTADAQH/MAoGCCqGSM49\r\n" \
								 "BAMCA0gAMEUCIHdctWRECwlVEMj8yEwJTVSRXNWud2jpyuOwKh1bAZ6ZAiEApghQ\r\n" \
								 "OOcAyflrXv1U0KPsJn7rw78KQoLWfvhf51Qv590=\r\n" \
								 "-----END CERTIFICATE-----\r\n";

#if defined(__ICCARM__)
#pragma language = extended
#endif

#undef __crypto_mem_dump
#define __crypto_mem_dump(start,size,prefix) do{ \
dbg_printf(prefix "\r\n"); \
  dump_bytes(start,size); \
}while(0)

// vector from NIST: AES ECB/CBC 256 bits :
// key,IV,AAD,tag start address needs to be 32bytes aligned
const unsigned char aes_256_key[32] __ALIGNED(32) = {
	0x60, 0x3D, 0xEB, 0x10, 0x15, 0xCA, 0x71, 0xBE,
	0x2B, 0x73, 0xAE, 0xF0, 0x85, 0x7D, 0x77, 0x81,
	0x1F, 0x35, 0x2C, 0x07, 0x3B, 0x61, 0x08, 0xD7,
	0x2D, 0x98, 0x10, 0xA3, 0x09, 0x14, 0xDF, 0xF4
};

const unsigned char aes_iv[16] __ALIGNED(32) = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};

const unsigned char aes_plaintext[16] = {
	0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
	0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a
};

static u8 cipher_result[1024] __ALIGNED(32);
void secure_api(char *(*a2)(void *), void *a3, int a4)
{
	int ret;
	if (a2 && a3) {
		memset(cipher_result, 0, sizeof(cipher_result));
		dbg_printf("AES 256 CBC test Decrypt \r\n");
		ret = crypto_init();
		if (SUCCESS != ret) {
			dbg_printf("crypto engine init failed \r\n");
			goto err;
		}
		ret = crypto_aes_cbc_init(aes_256_key, sizeof(aes_256_key));
		if (SUCCESS != ret) {
			dbg_printf("AES CBC init failed, ret = %d \r\n", ret);
			goto err;
		}
		ret = crypto_aes_cbc_decrypt(a3, a4, aes_iv, sizeof(aes_iv), cipher_result);
		if (SUCCESS != ret) {
			dbg_printf("AES CBC decrypt failed, ret = %d \r\n", ret);
			goto err;
		}
		if (memcmp(&aes_plaintext[0], cipher_result, a4) == 0) {
			dbg_printf("AES 256 decrypt result success \r\n");
			ret = SUCCESS;
		} else {
			dbg_printf("AES 256 decrypt result failed \r\n");
			__crypto_mem_dump((u8 *)&aes_plaintext[0], a4, "sw decrypt msg");
			__crypto_mem_dump(cipher_result, a4, "hw decrypt msg");
			ret = FAIL;
			goto err;
		}
#if defined(__ICCARM__)
		char *(__cmse_nonsecure_call * func)(void *d);
		func = (char *(__cmse_nonsecure_call *)(void *))a2;
#else
		char *__attribute__((cmse_nonsecure_call))(*func)(void *d);
		func = cmse_nsfptr_create(a2);
		if (cmse_is_nsfptr(func))
#endif
		{
			rt_printfl("Calling NS API from S world\n\r");
			rt_printfl("Received \"%s\" from NS world\n\r", func(a3));
		}
	}
err:
	return;
}

/**
  * Secure API which can be called from non-secure world
  */
uint32_t NS_ENTRY secure_api_gw(void *a0, int a1, char *(*a2)(void *))
{
	rt_printfl("Calling S API from NS world\n\r");
	secure_api(a2, a0, a1);
	return 0;
}

static void *my_calloc(size_t nelements, size_t elementSize)
{
	size_t size;
	void *ptr = NULL;
	size = nelements * elementSize;
	ptr = pvPortMalloc(size);

	if (ptr) {
		memset(ptr, 0, size);
	}

	return ptr;
}

#define my_free		vPortFree
#define mbedtls_printf rt_printfl

#if 1//defined(MBEDTLS_SELF_TEST)

#define GCD_PAIR_COUNT  3

static const int gcd_pairs[GCD_PAIR_COUNT][3] = {
	{ 693, 609, 21 },
	{ 1764, 868, 28 },
	{ 768454923, 542167814, 1 }
};

/*
 * Checkup routine
 */
int mbedtls_mpi_self_test(int verbose)
{
	int ret, i;
	mbedtls_mpi A, E, N, X, Y, U, V;

	mbedtls_mpi_init(&A);
	mbedtls_mpi_init(&E);
	mbedtls_mpi_init(&N);
	mbedtls_mpi_init(&X);
	mbedtls_mpi_init(&Y);
	mbedtls_mpi_init(&U);
	mbedtls_mpi_init(&V);

	MBEDTLS_MPI_CHK(mbedtls_mpi_read_string(&A, 16,
											"EFE021C2645FD1DC586E69184AF4A31E" \
											"D5F53E93B5F123FA41680867BA110131" \
											"944FE7952E2517337780CB0DB80E61AA" \
											"E7C8DDC6C5C6AADEB34EB38A2F40D5E6"));

	MBEDTLS_MPI_CHK(mbedtls_mpi_read_string(&E, 16,
											"B2E7EFD37075B9F03FF989C7C5051C20" \
											"34D2A323810251127E7BF8625A4F49A5" \
											"F3E27F4DA8BD59C47D6DAABA4C8127BD" \
											"5B5C25763222FEFCCFC38B832366C29E"));

	MBEDTLS_MPI_CHK(mbedtls_mpi_read_string(&N, 16,
											"0066A198186C18C10B2F5ED9B522752A" \
											"9830B69916E535C8F047518A889A43A5" \
											"94B6BED27A168D31D4A52F88925AA8F5"));

	MBEDTLS_MPI_CHK(mbedtls_mpi_mul_mpi(&X, &A, &N));

	MBEDTLS_MPI_CHK(mbedtls_mpi_read_string(&U, 16,
											"602AB7ECA597A3D6B56FF9829A5E8B85" \
											"9E857EA95A03512E2BAE7391688D264A" \
											"A5663B0341DB9CCFD2C4C5F421FEC814" \
											"8001B72E848A38CAE1C65F78E56ABDEF" \
											"E12D3C039B8A02D6BE593F0BBBDA56F1" \
											"ECF677152EF804370C1A305CAF3B5BF1" \
											"30879B56C61DE584A0F53A2447A51E"));

	if (verbose != 0) {
		mbedtls_printf("  MPI test #1 (mul_mpi): ");
	}

	if (mbedtls_mpi_cmp_mpi(&X, &U) != 0) {
		if (verbose != 0) {
			mbedtls_printf("failed\n");
		}

		ret = 1;
		goto cleanup;
	}

	if (verbose != 0) {
		mbedtls_printf("passed\n");
	}

	MBEDTLS_MPI_CHK(mbedtls_mpi_div_mpi(&X, &Y, &A, &N));

	MBEDTLS_MPI_CHK(mbedtls_mpi_read_string(&U, 16,
											"256567336059E52CAE22925474705F39A94"));

	MBEDTLS_MPI_CHK(mbedtls_mpi_read_string(&V, 16,
											"6613F26162223DF488E9CD48CC132C7A" \
											"0AC93C701B001B092E4E5B9F73BCD27B" \
											"9EE50D0657C77F374E903CDFA4C642"));

	if (verbose != 0) {
		mbedtls_printf("  MPI test #2 (div_mpi): ");
	}

	if (mbedtls_mpi_cmp_mpi(&X, &U) != 0 ||
		mbedtls_mpi_cmp_mpi(&Y, &V) != 0) {
		if (verbose != 0) {
			mbedtls_printf("failed\n");
		}

		ret = 1;
		goto cleanup;
	}

	if (verbose != 0) {
		mbedtls_printf("passed\n");
	}

	MBEDTLS_MPI_CHK(mbedtls_mpi_exp_mod(&X, &A, &E, &N, NULL));

	MBEDTLS_MPI_CHK(mbedtls_mpi_read_string(&U, 16,
											"36E139AEA55215609D2816998ED020BB" \
											"BD96C37890F65171D948E9BC7CBAA4D9" \
											"325D24D6A3C12710F10A09FA08AB87"));

	if (verbose != 0) {
		mbedtls_printf("  MPI test #3 (exp_mod): ");
	}

	if (mbedtls_mpi_cmp_mpi(&X, &U) != 0) {
		if (verbose != 0) {
			mbedtls_printf("failed\n");
		}

		ret = 1;
		goto cleanup;
	}

	if (verbose != 0) {
		mbedtls_printf("passed\n");
	}

	MBEDTLS_MPI_CHK(mbedtls_mpi_inv_mod(&X, &A, &N));

	MBEDTLS_MPI_CHK(mbedtls_mpi_read_string(&U, 16,
											"003A0AAEDD7E784FC07D8F9EC6E3BFD5" \
											"C3DBA76456363A10869622EAC2DD84EC" \
											"C5B8A74DAC4D09E03B5E0BE779F2DF61"));

	if (verbose != 0) {
		mbedtls_printf("  MPI test #4 (inv_mod): ");
	}

	if (mbedtls_mpi_cmp_mpi(&X, &U) != 0) {
		if (verbose != 0) {
			mbedtls_printf("failed\n");
		}

		ret = 1;
		goto cleanup;
	}

	if (verbose != 0) {
		mbedtls_printf("passed\n");
	}

	if (verbose != 0) {
		mbedtls_printf("  MPI test #5 (simple gcd): ");
	}

	for (i = 0; i < GCD_PAIR_COUNT; i++) {
		MBEDTLS_MPI_CHK(mbedtls_mpi_lset(&X, gcd_pairs[i][0]));
		MBEDTLS_MPI_CHK(mbedtls_mpi_lset(&Y, gcd_pairs[i][1]));

		MBEDTLS_MPI_CHK(mbedtls_mpi_gcd(&A, &X, &Y));

		if (mbedtls_mpi_cmp_int(&A, gcd_pairs[i][2]) != 0) {
			if (verbose != 0) {
				mbedtls_printf("failed at %d\n", i);
			}

			ret = 1;
			goto cleanup;
		}
	}

	if (verbose != 0) {
		mbedtls_printf("passed\n");
	}

cleanup:

	if (ret != 0 && verbose != 0) {
		mbedtls_printf("Unexpected error, return code = %08X\n", ret);
	}

	mbedtls_mpi_free(&A);
	mbedtls_mpi_free(&E);
	mbedtls_mpi_free(&N);
	mbedtls_mpi_free(&X);
	mbedtls_mpi_free(&Y);
	mbedtls_mpi_free(&U);
	mbedtls_mpi_free(&V);

	if (verbose != 0) {
		mbedtls_printf("\n");
	}

	return (ret);
}

#endif /* MBEDTLS_SELF_TEST */

#if 1//defined(MBEDTLS_SELF_TEST)

/*
 * Checkup routine
 */
int mbedtls_ecp_self_test(int verbose)
{
	int ret;
	size_t i;
	mbedtls_ecp_group grp;
	mbedtls_ecp_point R, P;
	mbedtls_mpi m;
	unsigned long add_c_prev, dbl_c_prev, mul_c_prev;
	/* exponents especially adapted for secp192r1 */
	const char *exponents[] = {
		"000000000000000000000000000000000000000000000001", /* one */
		"FFFFFFFFFFFFFFFFFFFFFFFF99DEF836146BC9B1B4D22830", /* N - 1 */
		"5EA6F389A38B8BC81E767753B15AA5569E1782E30ABE7D25", /* random */
		"400000000000000000000000000000000000000000000000", /* one and zeros */
		"7FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF", /* all ones */
		"555555555555555555555555555555555555555555555555", /* 101010... */
	};

	mbedtls_ecp_group_init(&grp);
	mbedtls_ecp_point_init(&R);
	mbedtls_ecp_point_init(&P);
	mbedtls_mpi_init(&m);

	/* Use secp192r1 if available, or any available curve */
#if defined(MBEDTLS_ECP_DP_SECP192R1_ENABLED)
	MBEDTLS_MPI_CHK(mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_SECP192R1));
#else
	MBEDTLS_MPI_CHK(mbedtls_ecp_group_load(&grp, mbedtls_ecp_curve_list()->grp_id));
#endif

	if (verbose != 0) {
		mbedtls_printf("  ECP test #1 (constant op_count, base point G): ");
	}

	/* Do a dummy multiplication first to trigger precomputation */
	MBEDTLS_MPI_CHK(mbedtls_mpi_lset(&m, 2));
	MBEDTLS_MPI_CHK(mbedtls_ecp_mul(&grp, &P, &m, &grp.G, NULL, NULL));
#if 0
	add_count = 0;
	dbl_count = 0;
	mul_count = 0;
#endif
	MBEDTLS_MPI_CHK(mbedtls_mpi_read_string(&m, 16, exponents[0]));
	MBEDTLS_MPI_CHK(mbedtls_ecp_mul(&grp, &R, &m, &grp.G, NULL, NULL));

	for (i = 1; i < sizeof(exponents) / sizeof(exponents[0]); i++) {
#if 0
		add_c_prev = add_count;
		dbl_c_prev = dbl_count;
		mul_c_prev = mul_count;
		add_count = 0;
		dbl_count = 0;
		mul_count = 0;
#endif
		MBEDTLS_MPI_CHK(mbedtls_mpi_read_string(&m, 16, exponents[i]));
		MBEDTLS_MPI_CHK(mbedtls_ecp_mul(&grp, &R, &m, &grp.G, NULL, NULL));
#if 0
		if (add_count != add_c_prev ||
			dbl_count != dbl_c_prev ||
			mul_count != mul_c_prev) {
			if (verbose != 0) {
				mbedtls_printf("failed (%u)\n", (unsigned int) i);
			}

			ret = 1;
			goto cleanup;
		}
#endif
	}

	if (verbose != 0) {
		mbedtls_printf("passed\n");
	}

	if (verbose != 0) {
		mbedtls_printf("  ECP test #2 (constant op_count, other point): ");
	}
	/* We computed P = 2G last time, use it */
#if 0
	add_count = 0;
	dbl_count = 0;
	mul_count = 0;
#endif
	MBEDTLS_MPI_CHK(mbedtls_mpi_read_string(&m, 16, exponents[0]));
	MBEDTLS_MPI_CHK(mbedtls_ecp_mul(&grp, &R, &m, &P, NULL, NULL));

	for (i = 1; i < sizeof(exponents) / sizeof(exponents[0]); i++) {
#if 0
		add_c_prev = add_count;
		dbl_c_prev = dbl_count;
		mul_c_prev = mul_count;
		add_count = 0;
		dbl_count = 0;
		mul_count = 0;
#endif
		MBEDTLS_MPI_CHK(mbedtls_mpi_read_string(&m, 16, exponents[i]));
		MBEDTLS_MPI_CHK(mbedtls_ecp_mul(&grp, &R, &m, &P, NULL, NULL));
#if 0
		if (add_count != add_c_prev ||
			dbl_count != dbl_c_prev ||
			mul_count != mul_c_prev) {
			if (verbose != 0) {
				mbedtls_printf("failed (%u)\n", (unsigned int) i);
			}

			ret = 1;
			goto cleanup;
		}
#endif
	}

	if (verbose != 0) {
		mbedtls_printf("passed\n");
	}

cleanup:

	if (ret < 0 && verbose != 0) {
		mbedtls_printf("Unexpected error, return code = %08X\n", ret);
	}

	mbedtls_ecp_group_free(&grp);
	mbedtls_ecp_point_free(&R);
	mbedtls_ecp_point_free(&P);
	mbedtls_mpi_free(&m);

	if (verbose != 0) {
		mbedtls_printf("\n");
	}

	return (ret);
}

#endif /* MBEDTLS_SELF_TEST */

void NS_ENTRY secure_main(void)
{
	rt_printfl("Start application in S world\n\r");

	mbedtls_platform_set_calloc_free(my_calloc, my_free);
	mbedtls_mpi_self_test(1);
	mbedtls_ecp_self_test(1); // cannot verify correctness, only verify work without fault
}
