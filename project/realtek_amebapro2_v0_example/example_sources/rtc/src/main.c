#include "rtc_api.h"
#include "wait_api.h"


void main(void)
{
	time_t seconds;
	struct tm *timeinfo;

	rtc_init();
	rtc_write(1256729737);  // Set RTC time to Wed, 28 Oct 2009 11:35:37


	while (1) {
		seconds = rtc_read();
		timeinfo = localtime(&seconds);

//        dbg_printf("Time as seconds since January 1, 1970 = %d\n\r", seconds);

		dbg_printf("Time as a basic string = %s\r", ctime(&seconds));

		dbg_printf("Time as a custom formatted string = %d-%d-%d %d:%d:%d\n\r",
				   timeinfo->tm_year, timeinfo->tm_mon, timeinfo->tm_mday, timeinfo->tm_hour,
				   timeinfo->tm_min, timeinfo->tm_sec);

		wait(1.0);
	}
}

