LWIP SNTP SHOWTIME EXAMPLE

Description:
Show system time maintained by time from NTP server and system tick.

Configuration:
1.Can Modify SNTP_SERVER_ADDRESS and SNTP_UPDATE_DELAY in sntp.c for NTP time update
For SNTP_UPDATE_DELAY to work, SNTP_STARTUP_DELAY should be defined
2.GCC:use CMD "make all EXAMPLE=sntp_showtime" to compile sntp_showtime example.

Execution:
Can make automatical Wi-Fi connection when booting by using wlan fast connect example.
A lwip ntp showtime example thread will be started automatically when booting.

[Supported List]
	Supported :
	    RTL8730A, RTL872XE