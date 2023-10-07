// Main file of the serial port project.
// NOTE: This file must not be changed.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "application_layer.h"

#include "link_layer.h" // TODO: Delete this when application layer is done

#define BAUDRATE 9600
#define N_TRIES 3
#define TIMEOUT 4

// Arguments:
//   $1: /dev/ttySxx
//   $2: tx | rx
//   $3: filename
int main(int argc, char *argv[])
{
    if (argc < 4)
    {
        printf("Usage: %s /dev/ttySxx tx|rx filename\n", argv[0]);
        exit(1);
    }

    const char *serialPort = argv[1];
    const char *role = argv[2];
    const char *filename = argv[3];

    printf("Starting link-layer protocol application\n"
           "  - Serial port: %s\n"
           "  - Role: %s\n"
           "  - Baudrate: %d\n"
           "  - Number of tries: %d\n"
           "  - Timeout: %d\n"
           "  - Filename: %s\n",
           serialPort,
           role,
           BAUDRATE,
           N_TRIES,
           TIMEOUT,
           filename);

    //applicationLayer(serialPort, role, BAUDRATE, N_TRIES, TIMEOUT, filename);
    //LinkLayer connectionParametersApp = {argv[1], (role == "r" : LlTx), BAUDRATE, N_TRIES, TIMEOUT};
    
    unsigned char *buf = (unsigned char *) malloc(4);
    buf[0] = 0x00;
    buf[1] = 0x01;
    buf[2] = 0x7E;
    buf[3] = 0x7D;
    llwrite(buf, 4);

    LinkLayer connectionParametersApp;
    strncpy(connectionParametersApp.serialPort, argv[1], sizeof(connectionParametersApp.serialPort)-1);
    connectionParametersApp.role = strcmp(role, "tx") ? LlRx : LlTx; 
    connectionParametersApp.baudRate = BAUDRATE;
    connectionParametersApp.nRetransmissions = N_TRIES;
    connectionParametersApp.timeout = TIMEOUT;
    
    int r;
    if(strcmp(role, "rx") == 0) r = llopen(connectionParametersApp);
    if(strcmp(role, "tx") == 0) r = llopen(connectionParametersApp);
    
    if(r < 0) printf("Some error !\n");

    if(strcmp(role, "rx") == 0) llclose(0);
    if(strcmp(role, "tx") == 0) llclose(0);

    return 0;
}
