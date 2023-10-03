#ifndef _DATA_LINK_H_
#define _DATA_LINK_H_

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 1
#define FRAME_SIZE 5

#define FLAG    0x7E
#define A_SEND  0x03
#define A_RECV  0x01
#define C_SET   0x03
#define C_UA    0x07

#define MAX_RET_ATTEMPTS 3 // Maximum number of retransmission attempts

#define TRANSMITTER 0
#define RECEIVER 1

enum state{
    START, 
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
};

struct linkLayer {
    char port[20]; /*Dispositivo /dev/ttySx, x = 0, 1*/
    int baudRate; /*Velocidade de transmissão*/
    unsigned int sequenceNumber; /*Número de sequência da trama: 0, 1*/
    unsigned int timeout; /*Valor do temporizador: 1 s*/
    unsigned int numTransmissions; /*Número de tentativas em caso de falha*/
    char frame[10]; /*Trama*/
};

int llopen(const char * port, unsigned char flag);

void alarmHandler(int signal);

void alarmDisable();

void print_answer(unsigned char *answer, int n);

void send_packet_command(int fd, unsigned char C, unsigned char A);

int connectFD(const char * port); // /dev/ttySx 

int disconnectFD(int fd);

#endif