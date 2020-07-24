#include "epoch_converter.h"
#include <string.h>

char str_time[9];
char str_date[6];

char * convert_unix_time(time_t unix_time)
{
	memset(str_time, 0x00, 9);
	struct tm ts;	
	ts = *localtime(&unix_time);
//	strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);
	strftime(str_time, sizeof(str_time), "%H:%M:%S", &ts);
	return str_time;
}

char * convert_unix_date(time_t unix_time)
{
	memset(str_date, 0x00, 6);
	struct tm ts;
	ts = *localtime(&unix_time);
	strftime(str_date, sizeof(str_date), "%d%m%y", &ts);
	return str_date;
}
