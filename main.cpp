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

int main(int argc, char **argv)
{
    int cam_threshold = 0;

    for(int i = 1; i < argc; ++i)
    {
        const int len = strlen(argv[i]);
        if(strcmp("-h", argv[i]) == 0 || strcmp("--help", argv[i]) == 0)
        {
            printf("Usage: %s [SWITCHES]\n"
                "    -t X                    - Set the detection threshold\n");
            return 0;
        }
        else if(strncmp("-t", argv[i], 2) == 0)
        {
            if(len > 2)
                cam_threshold = atoi(argv[i]+2);
            else if(i+1 < argc)
                cam_threshold = atoi(argv[++i]);
        }
        else
        {
            fprintf(stderr, "Unknown argument: %s\n", argv[i]);
            return 1;
        }
    }
    
    try
    {
        sComm.initialize();
        sTcpServer.initialize();
        sCamera.open(cam_threshold);
    }
    catch(const char* ex)
    {
        fprintf(stderr, ex);
        //return 255;
    }

    printf("Initialization complete.\n");


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