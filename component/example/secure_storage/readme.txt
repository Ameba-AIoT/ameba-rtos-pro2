secure_storage exmaple shows how to encrypt/decrypt user data in trustzone and store to flash.
secure efuse key is used for encrypt/decrypt. user data is decrypted and loaded into trustzone.
Please enable CONFIG_EXAMPLE_SECURE_STORAGE in platform_opts.h to run secure_storage example

/*For secure storage example */
#define CONFIG_EXAMPLE_SECURE_STORAGE	1

Support IC:
	Ameba-z2