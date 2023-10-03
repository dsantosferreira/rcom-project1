#include <stdio.h>
#include <stdlib.h>

#include "data_link/data_link.h"


// Argumentos
// 1: /dev/ttySxx
// 2: r ; s
int main(int argc, char *argv[])
{   
    if (argc < 3)
    {
        return -1;
    }
    unsigned char flag_f;
    const char *serialPort = argv[1];
    const char *flag = argv[2];
    if(flag[0] == 'r'){
        flag_f = 1;
    }
    if(flag[0] == 's'){
        flag_f = 0;
    }
    int r = llopen(serialPort, flag_f);
    if(r < 0) printf("Some error !\n");
    return 0;
}
