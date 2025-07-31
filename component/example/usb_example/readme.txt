This is an example to demo usb ethernet procedure for RTL8152B usb ethernet dongle.
It show how to use the driver to connect the network.

Test procedure:
1.Please modify the platform_opts.h
#define CONFIG_ETHERNET     1
2.Build the example with cmake.
3.Enter the UECM to enable the usb ethernet.
4.Wait the dhcp procedure to get the ip.
5.The exmaple can run the ping and streaming.
After you run the exmaple that you can enter the UVID to see the streaming.

[Supported List]
	Supported :
		AmebaPro2