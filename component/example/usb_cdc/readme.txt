This is an example for usb cdc procedure.
It show how to use the cdc.The demo show the log and loopback mode.

The dedault setup is log mode.

Test procedure:
Loopback mode:
Enter the key and it will return the same value.
Log mode:
This mode is the same as uart log.

#define USB_CONSOLE_LOG //Use the log mode that it is the same as the normal log service.
It has two config to setup the log mode.
#define CONSOLE_MODE 0X00 //It only output the log from USB CDC
#define REMOTE_MODE  0X01 //It show the log from uart and USB CDC
static int cdc_mode = REMOTE_MODE;//The default setup is remote mode.

If you disalbe the marco that it will run the loopback mode.

Note:
You can use the teraterm or device manager to check the serial port for usb cdc

[Supported List]
	Supported :
		AmebaPro2