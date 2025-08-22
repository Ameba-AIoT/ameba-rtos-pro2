SSL SERVER EXAMPLE

Description:
A simple SSL server which can reply for the https request.

Configuration:
[RTL8730A, RTL872XE]
1.Modify SERVER_PORT and response content in example_ssl_server.c based on your SSL server.

2.For MbedTLS:
	[..\mbedtls\config_rsa.h]
		#define MBEDTLS_CERTS_C
		#define MBEDTLS_SSL_SRV_C
		
3.Select which case to compile in app_example.
	void app_example(void)
	{
		example_ssl_server();
		//example_ssl_server_mebdtls_dtls();
	}

4.GCC:use CMD "make all EXAMPLE=ssl_server" to compile ssl_server example

[RTL8735B]
Only example_ssl_server is verified.
1. cmake with -DEXAMPLE=ssl_server
	ex. cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DEXAMPLE=ssl_server

Execution:
Can make automatical Wi-Fi connection when booting by using wlan fast connect example.
A ssl server example thread will be started automatically when booting.

[Supported List]
	Supported :
		RTL8730A, RTL872XE, RTL8735B

