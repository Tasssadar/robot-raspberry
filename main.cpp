#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <stdlib.h>

#include "comm.h"
#include "tcpserver.h"
#include "camera.h"
#include "util.h"

#define SLEEP_CONST 16

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

int main(int argc, char **argv)
{
    // Set line buffering for stdout and stderr
    setvbuf(stdout, NULL, _IOLBF, 256);
    setvbuf(stderr, NULL, _IOLBF, 256);

    for(int i = 1; i < argc; ++i)
    {
        const int len = strlen(argv[i]);
        if(strcmp("-h", argv[i]) == 0 || strcmp("--help", argv[i]) == 0)
        {
            printf("Usage: %s [SWITCHES]\n"
                "    -t X                    - Set the detection threshold\n",
                argv[0]);
            return 0;
        }
        else if(strncmp("-t", argv[i], 2) == 0)
        {
            int th = 0;
            if(len > 2)
                th = atoi(argv[i]+2);
            else if(i+1 < argc)
                th = atoi(argv[++i]);

            if(th > 0)
                sCamera.setThreshold(th);
        }
        else
        {
            LOGE("Unknown argument: %s", argv[i]);
            return 1;
        }
    }

    sComm.initialize();
    sTcpServer.initialize();
    sTunnelServer.initialize();

    LOGD("Initialization complete.");

    timespec last, curr;
    uint32_t diff = 0, prevSleepTime = 0;
    clock_gettime(CLOCK_MONOTONIC, &last);
    while(true)
    {
        clock_gettime(CLOCK_MONOTONIC, &curr);
        diff = timespec_diff(last, curr);

        // Process updates
        {
            sComm.update(diff);
            sTcpServer.update(diff);
            sTunnelServer.update(diff);
            sCamera.update(diff);
        }

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