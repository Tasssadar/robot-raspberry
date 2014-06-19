#include <time.h>
#include <stdio.h>
#include "util.h"

char *Util::getLogTime()
{
    char fmt[32];
    static char buff[32];

    struct timespec ts_now;
    struct tm tm_local;

    clock_gettime(CLOCK_REALTIME, &ts_now);
    localtime_r(&ts_now.tv_sec, &tm_local);

    strftime(fmt, sizeof(fmt), "[%d.%m.%y %H:%M:%S.%%03d]", &tm_local);
    snprintf(buff, sizeof(buff), fmt, ts_now.tv_nsec/1000000LLU);
    return buff;
}
