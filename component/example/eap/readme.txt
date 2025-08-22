802.1X EAP METHOD SUPPLICANT EXAMPLE

Description:
Use 802.1X EAP methods to connect to AP and authenticate with backend radius server.
Current supported methods are EAP-TLS, PEAPv0/EAP-MSCHAPv2, and EAP-TTLS/MSCHAPv2.

Configuration:
1.Modify the argument of example_eap() in example_entry.c to set which EAP methods you want to use.

2.Modify the connection config (ssid, identity, password, cert) in example_eap_config() of example_eap.c based on your server's setting.

3.[platform_opts.h]
	// Turn on/off the specified method
	# define CONFIG_ENABLE_PEAP	1
	# define CONFIG_ENABLE_TLS	1
	# define CONFIG_ENABLE_TTLS	1

	// If you want to verify the certificate of radius server, turn this on
	# define ENABLE_EAP_SSL_VERIFY_SERVER	1

4. get config arguments from wifi_eap_config.c
	add #define PRE_CONFIG_EAP in autoconf_eap.h

5. For RTL8730A, RTL872XE, lib_eap is not included by default, it need to be add to your project and then build.
	(a) Build lib_eap (for example:GCC-RELEASE\project_ap\asdk\make\network\makefile)
		make -C eap all
		make -C eap clean
	(b) Add lib_eap.a to project and modify makefile/IAR project setting to link it
		LINK_APP_LIB += $(ROOTDIR)/lib/application/lib_eap.a

6. For RTL8735B, how to build eap example
	(a) add the new folder to use the following instructions to build eap example
		cmake .. -G"Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake -DEXAMPLE=eap
		cmake --build . --target flash
    (b) enable component/example/eap/app_example.c about peap/tls/ttls to test


Execution:
An EAP connection thread will be started automatically when booting.

Trouble Shooting:
	ERROR: [eap_recvd]Malloc failed
	Solution: Increase the FreeRTOS heap in FreeRTOSConfig.h, 
       	#define configTOTAL_HEAP_SIZE	( ( size_t ) ( XX * 1024 ) )

Note:
Please make sure the lib_wps, polarssl/mbedtls, ssl_ram_map are also builded.
If the connection failed, you can try the following directions to make it work:
1. Make sure the config_rsa.h of PolarSSL/MbedTLS include the SSL/TLS cipher suite supported by radius server.
2. Set a larger value to SSL_MAX_CONTENT_LEN in config_rsa.h
3. Try to change using SW crypto instead of HW crypto.


[Supported List]
	Supported :
	    RTL8730A, RTL872XE, RTL8735B
		
Note:
For RTL8730A, RTL872XE, lib_eap is not included by default, it need to be add to your project and then build.
For GCC, please modify \asdk\makefile to link lib_eap.a, should add "LINK_APP_LIB += $(ROOTDIR)/lib/application/lib_eap.a" in make file.
For IAR, please add lib_eap.a to the lib fold in km4_application project.