SSL DOWNLOAD EXAMPLE

Description:
Download file from Web server via https.

Configuration:
1.Modify SERVER_HOST, SERVER_PORT and RESOURCE in example_ssl_download.c based on your SSL server.

2.Modify MBEDTLS_SSL_MAX_CONTENT_LEN in SSL config and configTOTAL_HEAP_SIZE in freertos config for large size file.
	If the transmitted file size is larger than 16kbytes, MBEDTLS_SSL_MAX_CONTENT_LEN should be set to 16384.
	FreeRTOS heap may be increased for ssl buffer allocation.
	(ex. If using 16kbytes * 2 for ssl input/output buffer, heap should be increased from 60kbytes to 80kbytes.)
	The definition of MBEDTLS_SSL_MAX_CONTENT_LEN
	If use mbedtls-2.16.6, define in config_rsa.h
	If use mbedtls-2.28.1, define in mbedtls_config.h
4.Use -DEXAMPLE=ssl_download to generate Make file


Execution:
Can make automatical Wi-Fi connection when booting by using wlan fast connect example.
A ssl download example thread will be started automatically when booting.

[Supported List]
	Supported :
	    RTL8730A (default CONFIG_USE_MBEDTLS), RTL872XE