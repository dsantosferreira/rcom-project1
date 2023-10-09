// Main file of the serial port project.
// NOTE: This file must not be changed.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "application_layer.h"

#include "link_layer.h" // TODO: Delete this when application layer is done

#define BAUDRATE 960
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

    // applicationLayer(serialPort, role, BAUDRATE, N_TRIES, TIMEOUT, filename);
    


    LinkLayer connectionParametersApp;
    strncpy(connectionParametersApp.serialPort, argv[1], sizeof(connectionParametersApp.serialPort)-1);
    connectionParametersApp.role = strcmp(role, "tx") ? LlRx : LlTx; 
    connectionParametersApp.baudRate = BAUDRATE;
    connectionParametersApp.nRetransmissions = N_TRIES;
    connectionParametersApp.timeout = TIMEOUT;
    
    if(strcmp(role, "rx") == 0) llopen(connectionParametersApp);
    if(strcmp(role, "tx") == 0) llopen(connectionParametersApp);
    
    FILE *file = NULL;
    unsigned char *buffer = (unsigned char *)malloc(BAUDRATE);
    size_t bytesRead = 0;


    if (strcmp(role, "tx") == 0) {
        file = fopen(filename, "rb");
        while ((bytesRead = fread(buffer, 1, BAUDRATE, file))) {
            printf("Estou no while\n");
            llwrite(buffer, bytesRead);
        }
    } else {
        file = fopen(filename, "w");
        unsigned char *packet = (unsigned char *) malloc(BAUDRATE);
        while ((bytesRead = llread(packet))) {
            for(int i = 0; i < bytesRead; i++) 
            {
                printf("buf[%d] = 0x%02X\n", i, packet[i]);
            }

            fwrite(packet, 1, bytesRead, file);

            if (bytesRead < BAUDRATE) break;
        }
    }

    if(strcmp(role, "rx") == 0) llclose(0);
    if(strcmp(role, "tx") == 0) llclose(0);

    return 0;
}
