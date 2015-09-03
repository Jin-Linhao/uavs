
#include <string.h>

#include <stdio.h>
#include <errno.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ssuwb.h"

int main(int argc, char *argv[])
{
    int* dis;

    if (argc!=3)
    {
        printf("error usage: demo /dev/ttyACM0 serial/usb\n");
        exit(0);
    }

    ssUWB uwb(argv[1],argv[2]);

    while(1)
    {

        dis=uwb.measure();

        printf("dis=%d dis=%d dis=%d dis=%d\n", dis[0],dis[1],dis[2],dis[3]);

    }
    return 0;
}


