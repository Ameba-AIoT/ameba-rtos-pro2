#undef CONFIG_SSL_RSA
#define CONFIG_SSL_RSA          1

#include "rom_ssl_ram_map.h"
#include "platform_opts.h"
#define RTL_HW_CRYPTO

#if defined (CONFIG_PLATFORM_AMEBAD2)
#define SUPPORT_HW_SW_CRYPTO
#if defined (ARM_CORE_CA7)
#undef CONFIG_SSL_RSA
#define CONFIG_SSL_RSA          0
#endif
#endif

/* RTL_CRYPTO_FRAGMENT should be less than 16000, and should be 16bytes-aligned */
#define RTL_CRYPTO_FRAGMENT               15360

#if defined(CONFIG_PLATFORM_8710C)
//#define SUPPORT_HW_SSL_HMAC_SHA256
#endif

#define MBEDTLS_VERSION_CONVERT(a,b,c)	(((a) << 16) + ((b) << 8) + (c))
#define MBEDTLS_VERSION		MBEDTLS_VERSION_CONVERT(2,4,0)

#if defined(CONFIG_SSL_ROM)
#include <section_config.h>
#include "platform_stdlib.h"
#include "mbedtls/config_rom.h"
#define SUPPORT_HW_SW_CRYPTO
#elif defined(CONFIG_BAIDU_DUER) && CONFIG_BAIDU_DUER
#define CONFIG_SSL_RSA          0
#include "baidu_ca_mbedtls_config.h"
#elif defined(ENABLE_AMAZON_COMMON)
#undef CONFIG_SSL_RSA
#define CONFIG_SSL_RSA          0
#include "platform_stdlib.h"
#include "mbedtls/config_amazon.h"
#elif (defined(CONFIG_EXAMPLE_FFS) && CONFIG_EXAMPLE_FFS)
#include "platform_stdlib.h"
#include "mbedtls/config_amazon_ffs.h"
#elif defined(CONFIG_SSL_RSA) && CONFIG_SSL_RSA
#include "platform_stdlib.h"
#include "mbedtls/config_rsa.h"
#else
#include "platform_stdlib.h"
#include "mbedtls/config_all.h"
#endif
