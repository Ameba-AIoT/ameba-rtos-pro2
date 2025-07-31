#include <time.h>

int gettimeofday(struct timeval *tv, struct timezone *tz)
{
	if (tv == NULL) {
		return -1;
	}

	tv->tv_sec = (long int) time((time_t *) NULL);
	tv->tv_usec = 0L;

	if (tz != NULL) {
		const time_t timer = tv->tv_sec;
		struct tm tm;
		const struct tm *tmp;

		const long int save_timezone = _timezone;
		const long int save_daylight = _daylight ;
		char *save_tzname[2];
		save_tzname[0] = _tzname[0];
		save_tzname[1] = _tzname[1];

		tmp = localtime_r(&timer, &tm);

		tz->tz_minuteswest = _timezone / 60;
		tz->tz_dsttime = _daylight;

		_timezone  = save_timezone;
		_daylight  = save_daylight;
		_tzname[0] = save_tzname[0];
		_tzname[1] = save_tzname[1];

		if (tmp == NULL) {
			return -1;
		}
	}

	return 0;
}