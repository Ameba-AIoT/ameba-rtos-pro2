/*
 *  FIPS-197 compliant AES implementation
 *
 *  Copyright The Mbed TLS Contributors
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
/*
 *  The AES block cipher was designed by Vincent Rijmen and Joan Daemen.
 *
 *  http://csrc.nist.gov/encryption/aes/rijndael/Rijndael.pdf
 *  http://csrc.nist.gov/publications/fips/fips197/fips-197.pdf
 */

#include "common.h"

#if defined(MBEDTLS_AES_ALT)

#include <string.h>

#include "mbedtls/aes.h"
#include "mbedtls/platform.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/error.h"

#include "crypto_api.h"
#include "device_lock.h"

#if defined(CONFIG_BUILD_SECURE) && (CONFIG_BUILD_SECURE == 1)
extern void ns_device_mutex_lock_call(uint32_t device);
extern void ns_device_mutex_unlock_call(uint32_t device);
#define device_mutex_lock ns_device_mutex_lock_call
#define device_mutex_unlock ns_device_mutex_unlock_call
#endif

void mbedtls_aes_init(mbedtls_aes_context *ctx)
{
    memset(ctx, 0, sizeof(mbedtls_aes_context));
}

void mbedtls_aes_free(mbedtls_aes_context *ctx)
{
    if (ctx == NULL) {
        return;
    }

    mbedtls_platform_zeroize(ctx, sizeof(mbedtls_aes_context));
}

#if defined(MBEDTLS_CIPHER_MODE_XTS)
void mbedtls_aes_xts_init(mbedtls_aes_xts_context *ctx)
{
    mbedtls_aes_init(&ctx->crypt);
    mbedtls_aes_init(&ctx->tweak);
}

void mbedtls_aes_xts_free(mbedtls_aes_xts_context *ctx)
{
    if (ctx == NULL) {
        return;
    }

    mbedtls_aes_free(&ctx->crypt);
    mbedtls_aes_free(&ctx->tweak);
}
#endif /* MBEDTLS_CIPHER_MODE_XTS */

/* Some implementations need the round keys to be aligned.
 * Return an offset to be added to buf, such that (buf + offset) is
 * correctly aligned.
 * Note that the offset is in units of elements of buf, i.e. 32-bit words,
 * i.e. an offset of 1 means 4 bytes and so on.
 */
#if (defined(MBEDTLS_PADLOCK_C) && defined(MBEDTLS_HAVE_X86)) ||        \
    (defined(MBEDTLS_AESNI_C) && MBEDTLS_AESNI_HAVE_CODE == 2)
#define MAY_NEED_TO_ALIGN
#endif
static unsigned mbedtls_aes_rk_offset(uint32_t *buf)
{
#if defined(MAY_NEED_TO_ALIGN)
    int align_16_bytes = 0;

#if defined(MBEDTLS_PADLOCK_C) && defined(MBEDTLS_HAVE_X86)
    if (aes_padlock_ace == -1) {
        aes_padlock_ace = mbedtls_padlock_has_support(MBEDTLS_PADLOCK_ACE);
    }
    if (aes_padlock_ace) {
        align_16_bytes = 1;
    }
#endif

#if defined(MBEDTLS_AESNI_C) && MBEDTLS_AESNI_HAVE_CODE == 2
    if (mbedtls_aesni_has_support(MBEDTLS_AESNI_AES)) {
        align_16_bytes = 1;
    }
#endif

    if (align_16_bytes) {
        /* These implementations needs 16-byte alignment
         * for the round key array. */
        unsigned delta = ((uintptr_t) buf & 0x0000000fU) / 4;
        if (delta == 0) {
            return 0;
        } else {
            return 4 - delta; // 16 bytes = 4 uint32_t
        }
    }
#else /* MAY_NEED_TO_ALIGN */
    (void) buf;
#endif /* MAY_NEED_TO_ALIGN */

    return 0;
}

/*
 * AES key schedule (encryption)
 */
int mbedtls_aes_setkey_enc(mbedtls_aes_context *ctx, const unsigned char *key,
                           unsigned int keybits)
{
    switch (keybits) {
        case 128: ctx->nr = 10; break;
        case 192: ctx->nr = 12; break;
        case 256: ctx->nr = 14; break;
        default: return MBEDTLS_ERR_AES_INVALID_KEY_LENGTH;
    }

    ctx->rk = ctx->buf;
    memcpy( ctx->rk, key, keybits / 8 );

    return 0;
}

/*
 * AES key schedule (decryption)
 */
int mbedtls_aes_setkey_dec(mbedtls_aes_context *ctx, const unsigned char *key,
                           unsigned int keybits)
{
    int ret;

    /* Also checks keybits */
    if ((ret = mbedtls_aes_setkey_enc(ctx, key, keybits)) != 0) {
        goto exit;
    }

exit:

    return ret;
}

#if defined(MBEDTLS_CIPHER_MODE_XTS)
static int mbedtls_aes_xts_decode_keys(const unsigned char *key,
                                       unsigned int keybits,
                                       const unsigned char **key1,
                                       unsigned int *key1bits,
                                       const unsigned char **key2,
                                       unsigned int *key2bits)
{
    const unsigned int half_keybits = keybits / 2;
    const unsigned int half_keybytes = half_keybits / 8;

    switch (keybits) {
        case 256: break;
        case 512: break;
        default: return MBEDTLS_ERR_AES_INVALID_KEY_LENGTH;
    }

    *key1bits = half_keybits;
    *key2bits = half_keybits;
    *key1 = &key[0];
    *key2 = &key[half_keybytes];

    return 0;
}

int mbedtls_aes_xts_setkey_enc(mbedtls_aes_xts_context *ctx,
                               const unsigned char *key,
                               unsigned int keybits)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    const unsigned char *key1, *key2;
    unsigned int key1bits, key2bits;

    ret = mbedtls_aes_xts_decode_keys(key, keybits, &key1, &key1bits,
                                      &key2, &key2bits);
    if (ret != 0) {
        return ret;
    }

    /* Set the tweak key. Always set tweak key for the encryption mode. */
    ret = mbedtls_aes_setkey_enc(&ctx->tweak, key2, key2bits);
    if (ret != 0) {
        return ret;
    }

    /* Set crypt key for encryption. */
    return mbedtls_aes_setkey_enc(&ctx->crypt, key1, key1bits);
}

int mbedtls_aes_xts_setkey_dec(mbedtls_aes_xts_context *ctx,
                               const unsigned char *key,
                               unsigned int keybits)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    const unsigned char *key1, *key2;
    unsigned int key1bits, key2bits;

    ret = mbedtls_aes_xts_decode_keys(key, keybits, &key1, &key1bits,
                                      &key2, &key2bits);
    if (ret != 0) {
        return ret;
    }

    /* Set the tweak key. Always set tweak key for encryption. */
    ret = mbedtls_aes_setkey_enc(&ctx->tweak, key2, key2bits);
    if (ret != 0) {
        return ret;
    }

    /* Set crypt key for decryption. */
    return mbedtls_aes_setkey_dec(&ctx->crypt, key1, key1bits);
}
#endif /* MBEDTLS_CIPHER_MODE_XTS */

/*
 * AES-ECB block encryption/decryption
 */
int mbedtls_aes_crypt_ecb(mbedtls_aes_context *ctx,
                          int mode,
                          const unsigned char input[16],
                          unsigned char output[16])
{
    unsigned char key_buf[32 + 32 + 32], *key_buf_aligned;

    if (mode != MBEDTLS_AES_ENCRYPT && mode != MBEDTLS_AES_DECRYPT) {
        return MBEDTLS_ERR_AES_BAD_INPUT_DATA;
    }

    key_buf_aligned = (unsigned char *) (((unsigned int) key_buf + 32) / 32 * 32);
    memcpy( key_buf_aligned, ctx->rk, ((ctx->nr - 6) * 4) );

    device_mutex_lock( RT_DEV_LOCK_CRYPTO );
    rtl_crypto_aes_ecb_init( key_buf_aligned, ((ctx->nr - 6) * 4) );

    if( mode == MBEDTLS_AES_ENCRYPT )
        rtl_crypto_aes_ecb_encrypt( input, 16, NULL, 0, output );
    else
        rtl_crypto_aes_ecb_decrypt( input, 16, NULL, 0, output );

    device_mutex_unlock( RT_DEV_LOCK_CRYPTO );

    return( 0 );
}

#if defined(MBEDTLS_CIPHER_MODE_CBC)
/*
 * AES-CBC buffer encryption/decryption
 */
int mbedtls_aes_crypt_cbc(mbedtls_aes_context *ctx,
                          int mode,
                          size_t length,
                          unsigned char iv[16],
                          const unsigned char *input,
                          unsigned char *output)
{
    unsigned char key_buf[32 + 32 + 32], *key_buf_aligned;
    unsigned char iv_buf[32 + 16 + 32], *iv_buf_aligned, iv_tmp[16];
    size_t length_done = 0;

    if (mode != MBEDTLS_AES_ENCRYPT && mode != MBEDTLS_AES_DECRYPT) {
        return MBEDTLS_ERR_AES_BAD_INPUT_DATA;
    }

    if (length % 16) {
        return MBEDTLS_ERR_AES_INVALID_INPUT_LENGTH;
    }

    if( length > 0 )
    {
        key_buf_aligned = (unsigned char *) (((unsigned int) key_buf + 32) / 32 * 32);
        memcpy(key_buf_aligned, ctx->rk, ((ctx->nr - 6) * 4));
        iv_buf_aligned = (unsigned char *) (((unsigned int) iv_buf + 32) / 32 * 32);
        memcpy(iv_buf_aligned, iv, 16);


        device_mutex_lock( RT_DEV_LOCK_CRYPTO );
        rtl_crypto_aes_cbc_init( key_buf_aligned, ((ctx->nr - 6) * 4) );

        if( mode == MBEDTLS_AES_DECRYPT )
        {
            while( (length - length_done) > RTL_CRYPTO_FRAGMENT )
            {
                memcpy( iv_tmp, (input + length_done + RTL_CRYPTO_FRAGMENT - 16), 16 );
                rtl_crypto_aes_cbc_decrypt( input + length_done, RTL_CRYPTO_FRAGMENT, iv_buf_aligned, 16, output + length_done );
                memcpy( iv_buf_aligned, iv_tmp, 16 );
                length_done += RTL_CRYPTO_FRAGMENT;
            }

            memcpy( iv_tmp, (input + length - 16), 16 );
            rtl_crypto_aes_cbc_decrypt( input + length_done, length - length_done, iv_buf_aligned, 16, output + length_done );
            memcpy( iv, iv_tmp, 16 );
        }
        else
        {
            while( (length - length_done) > RTL_CRYPTO_FRAGMENT )
            {
                rtl_crypto_aes_cbc_encrypt( input + length_done, RTL_CRYPTO_FRAGMENT, iv_buf_aligned, 16, output + length_done );
                memcpy( iv_buf_aligned, (output + length_done + RTL_CRYPTO_FRAGMENT - 16), 16 );
                length_done += RTL_CRYPTO_FRAGMENT;
            }

            rtl_crypto_aes_cbc_encrypt( input + length_done, length - length_done, iv_buf_aligned, 16, output + length_done );
            memcpy( iv, (output + length - 16), 16 );
        }

        device_mutex_unlock( RT_DEV_LOCK_CRYPTO );
    }

    return( 0 );
}
#endif /* MBEDTLS_CIPHER_MODE_CBC */

#if defined(MBEDTLS_CIPHER_MODE_XTS)

typedef unsigned char mbedtls_be128[16];

/*
 * GF(2^128) multiplication function
 *
 * This function multiplies a field element by x in the polynomial field
 * representation. It uses 64-bit word operations to gain speed but compensates
 * for machine endianness and hence works correctly on both big and little
 * endian machines.
 */
static void mbedtls_gf128mul_x_ble(unsigned char r[16],
                                   const unsigned char x[16])
{
    uint64_t a, b, ra, rb;

    a = MBEDTLS_GET_UINT64_LE(x, 0);
    b = MBEDTLS_GET_UINT64_LE(x, 8);

    ra = (a << 1)  ^ 0x0087 >> (8 - ((b >> 63) << 3));
    rb = (a >> 63) | (b << 1);

    MBEDTLS_PUT_UINT64_LE(ra, r, 0);
    MBEDTLS_PUT_UINT64_LE(rb, r, 8);
}

/*
 * AES-XTS buffer encryption/decryption
 */
int mbedtls_aes_crypt_xts(mbedtls_aes_xts_context *ctx,
                          int mode,
                          size_t length,
                          const unsigned char data_unit[16],
                          const unsigned char *input,
                          unsigned char *output)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t blocks = length / 16;
    size_t leftover = length % 16;
    unsigned char tweak[16];
    unsigned char prev_tweak[16];
    unsigned char tmp[16];

    if (mode != MBEDTLS_AES_ENCRYPT && mode != MBEDTLS_AES_DECRYPT) {
        return MBEDTLS_ERR_AES_BAD_INPUT_DATA;
    }

    /* Data units must be at least 16 bytes long. */
    if (length < 16) {
        return MBEDTLS_ERR_AES_INVALID_INPUT_LENGTH;
    }

    /* NIST SP 800-38E disallows data units larger than 2**20 blocks. */
    if (length > (1 << 20) * 16) {
        return MBEDTLS_ERR_AES_INVALID_INPUT_LENGTH;
    }

    /* Compute the tweak. */
    ret = mbedtls_aes_crypt_ecb(&ctx->tweak, MBEDTLS_AES_ENCRYPT,
                                data_unit, tweak);
    if (ret != 0) {
        return ret;
    }

    while (blocks--) {
        if (leftover && (mode == MBEDTLS_AES_DECRYPT) && blocks == 0) {
            /* We are on the last block in a decrypt operation that has
             * leftover bytes, so we need to use the next tweak for this block,
             * and this tweak for the leftover bytes. Save the current tweak for
             * the leftovers and then update the current tweak for use on this,
             * the last full block. */
            memcpy(prev_tweak, tweak, sizeof(tweak));
            mbedtls_gf128mul_x_ble(tweak, tweak);
        }

        mbedtls_xor(tmp, input, tweak, 16);

        ret = mbedtls_aes_crypt_ecb(&ctx->crypt, mode, tmp, tmp);
        if (ret != 0) {
            return ret;
        }

        mbedtls_xor(output, tmp, tweak, 16);

        /* Update the tweak for the next block. */
        mbedtls_gf128mul_x_ble(tweak, tweak);

        output += 16;
        input += 16;
    }

    if (leftover) {
        /* If we are on the leftover bytes in a decrypt operation, we need to
         * use the previous tweak for these bytes (as saved in prev_tweak). */
        unsigned char *t = mode == MBEDTLS_AES_DECRYPT ? prev_tweak : tweak;

        /* We are now on the final part of the data unit, which doesn't divide
         * evenly by 16. It's time for ciphertext stealing. */
        size_t i;
        unsigned char *prev_output = output - 16;

        /* Copy ciphertext bytes from the previous block to our output for each
         * byte of ciphertext we won't steal. */
        for (i = 0; i < leftover; i++) {
            output[i] = prev_output[i];
        }

        /* Copy the remainder of the input for this final round. */
        mbedtls_xor(tmp, input, t, leftover);

        /* Copy ciphertext bytes from the previous block for input in this
         * round. */
        mbedtls_xor(tmp + i, prev_output + i, t + i, 16 - i);

        ret = mbedtls_aes_crypt_ecb(&ctx->crypt, mode, tmp, tmp);
        if (ret != 0) {
            return ret;
        }

        /* Write the result back to the previous block, overriding the previous
         * output we copied. */
        mbedtls_xor(prev_output, tmp, t, 16);
    }

    return 0;
}
#endif /* MBEDTLS_CIPHER_MODE_XTS */

#if defined(MBEDTLS_CIPHER_MODE_CFB)
/*
 * AES-CFB128 buffer encryption/decryption
 */
int mbedtls_aes_crypt_cfb128(mbedtls_aes_context *ctx,
                             int mode,
                             size_t length,
                             size_t *iv_off,
                             unsigned char iv[16],
                             const unsigned char *input,
                             unsigned char *output)
{
    int c;
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t n;

    if (mode != MBEDTLS_AES_ENCRYPT && mode != MBEDTLS_AES_DECRYPT) {
        return MBEDTLS_ERR_AES_BAD_INPUT_DATA;
    }

    n = *iv_off;

    if (n > 15) {
        return MBEDTLS_ERR_AES_BAD_INPUT_DATA;
    }

    if (mode == MBEDTLS_AES_DECRYPT) {
        while (length--) {
            if (n == 0) {
                ret = mbedtls_aes_crypt_ecb(ctx, MBEDTLS_AES_ENCRYPT, iv, iv);
                if (ret != 0) {
                    goto exit;
                }
            }

            c = *input++;
            *output++ = (unsigned char) (c ^ iv[n]);
            iv[n] = (unsigned char) c;

            n = (n + 1) & 0x0F;
        }
    } else {
        while (length--) {
            if (n == 0) {
                ret = mbedtls_aes_crypt_ecb(ctx, MBEDTLS_AES_ENCRYPT, iv, iv);
                if (ret != 0) {
                    goto exit;
                }
            }

            iv[n] = *output++ = (unsigned char) (iv[n] ^ *input++);

            n = (n + 1) & 0x0F;
        }
    }

    *iv_off = n;
    ret = 0;

exit:
    return ret;
}

/*
 * AES-CFB8 buffer encryption/decryption
 */
int mbedtls_aes_crypt_cfb8(mbedtls_aes_context *ctx,
                           int mode,
                           size_t length,
                           unsigned char iv[16],
                           const unsigned char *input,
                           unsigned char *output)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    unsigned char c;
    unsigned char ov[17];

    if (mode != MBEDTLS_AES_ENCRYPT && mode != MBEDTLS_AES_DECRYPT) {
        return MBEDTLS_ERR_AES_BAD_INPUT_DATA;
    }
    while (length--) {
        memcpy(ov, iv, 16);
        ret = mbedtls_aes_crypt_ecb(ctx, MBEDTLS_AES_ENCRYPT, iv, iv);
        if (ret != 0) {
            goto exit;
        }

        if (mode == MBEDTLS_AES_DECRYPT) {
            ov[16] = *input;
        }

        c = *output++ = (unsigned char) (iv[0] ^ *input++);

        if (mode == MBEDTLS_AES_ENCRYPT) {
            ov[16] = c;
        }

        memcpy(iv, ov + 1, 16);
    }
    ret = 0;

exit:
    return ret;
}
#endif /* MBEDTLS_CIPHER_MODE_CFB */

#if defined(MBEDTLS_CIPHER_MODE_OFB)
/*
 * AES-OFB (Output Feedback Mode) buffer encryption/decryption
 */
int mbedtls_aes_crypt_ofb(mbedtls_aes_context *ctx,
                          size_t length,
                          size_t *iv_off,
                          unsigned char iv[16],
                          const unsigned char *input,
                          unsigned char *output)
{
    int ret = 0;
    size_t n;

    n = *iv_off;

    if (n > 15) {
        return MBEDTLS_ERR_AES_BAD_INPUT_DATA;
    }

    while (length--) {
        if (n == 0) {
            ret = mbedtls_aes_crypt_ecb(ctx, MBEDTLS_AES_ENCRYPT, iv, iv);
            if (ret != 0) {
                goto exit;
            }
        }
        *output++ =  *input++ ^ iv[n];

        n = (n + 1) & 0x0F;
    }

    *iv_off = n;

exit:
    return ret;
}
#endif /* MBEDTLS_CIPHER_MODE_OFB */

#if defined(MBEDTLS_CIPHER_MODE_CTR)
/*
 * AES-CTR buffer encryption/decryption
 */
int mbedtls_aes_crypt_ctr(mbedtls_aes_context *ctx,
                          size_t length,
                          size_t *nc_off,
                          unsigned char nonce_counter[16],
                          unsigned char stream_block[16],
                          const unsigned char *input,
                          unsigned char *output)
{
    int c, i;
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t n;

    n = *nc_off;

    if (n > 0x0F) {
        return MBEDTLS_ERR_AES_BAD_INPUT_DATA;
    }

    while (length--) {
        if (n == 0) {
            ret = mbedtls_aes_crypt_ecb(ctx, MBEDTLS_AES_ENCRYPT, nonce_counter, stream_block);
            if (ret != 0) {
                goto exit;
            }

            for (i = 16; i > 0; i--) {
                if (++nonce_counter[i - 1] != 0) {
                    break;
                }
            }
        }
        c = *input++;
        *output++ = (unsigned char) (c ^ stream_block[n]);

        n = (n + 1) & 0x0F;
    }

    *nc_off = n;
    ret = 0;

exit:
    return ret;
}
#endif /* MBEDTLS_CIPHER_MODE_CTR */

#endif /* MBEDTLS_AES_ALT */
