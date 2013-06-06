#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <stdlib.h>

#include "comm.h"
#include "tcpserver.h"

#define SLEEP_CONST 50

static uint32_t timespec_diff(const timespec &f, const timespec &s)
{
    uint32_t res = 0;
    if(s.tv_nsec-f.tv_nsec < 0)
    {
        res = (s.tv_sec-f.tv_sec-1)*1000;
        res += 1000 + ((s.tv_nsec-f.tv_nsec)/1000000);
    }
    else
    {
        res = (s.tv_sec-f.tv_sec)*1000;
        res += (s.tv_nsec-f.tv_nsec)/1000000;
    }
    return res;
}

int main()
{
    try
    {
        sComm.initialize();
        sTcpServer.initialize();
    }
    catch(const char* ex)
    {
        fprintf(stderr, ex);
        return 255;
    }


    timespec last, curr;
    uint32_t diff = 0, prevSleepTime = 0;
    clock_gettime(CLOCK_MONOTONIC, &last);
    while(true)
    {
        clock_gettime(CLOCK_MONOTONIC, &curr);
        diff = timespec_diff(last, curr);

        {
            sComm.update(diff);
            sTcpServer.update(diff);
        }

        sTcpServer.write("ahoj\n", 5);

        last = curr;
        if(diff <= SLEEP_CONST+prevSleepTime)
        {
            prevSleepTime = SLEEP_CONST+prevSleepTime-diff;
            usleep(prevSleepTime*1000);
        }
        else
            prevSleepTime = 0;
    }

    return 0;
}