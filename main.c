#include <stdio.h>
#include <stdlib.h>
#include "data_link/data_link.h"


int main(int argc, char *argv[])
{   
    if (argc < 3)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS10 s\n",
               argv[0],
               argv[0]);
        exit(-1);
    }

    int r;
    const char *serialPort = argv[1];
    const char *flag = argv[2];
    if(flag[0] == 'r') r = llopen(serialPort, RECEIVER);
    if(flag[0] == 's') r = llopen(serialPort, TRANSMITTER);
    
    if(r < 0) printf("Some error !\n");

    return 0;
}

