// Link layer protocol implementation

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

#include "link_layer.h"


// MISC
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 1
#define FRAME_SIZE 5

#define FLAG        0x7E
#define ESC         0x7D
#define ESC_FLAG    0x5E
#define ESC_ESC     0x5D
#define A_SEND      0x03
#define A_RECV      0x01
#define C_SET       0x03
#define C_UA        0x07
#define C_DISC      0x0B

#define RR0         0x05
#define RR1         0x85
#define REJ0        0x01
#define REJ1        0x81

enum state{
    START, 
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    BCC2_OK,
    STOP
};

LinkLayer connectionParameters;
struct termios oldtio;
int alarmEnabled = FALSE;
int alarmCount = 0;
int fd;
unsigned char C_Ns = 0; // Ns

int connectFD(const char * port)
{
    fd = open(port, O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        perror(port);
        return -1;
    }

    
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        return -1;
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    newtio.c_lflag = 0;

    // TODO: Check these values for transmiter and receiver. Maybe pass the timeout as a parameter for VTIME?
    newtio.c_cc[VTIME] = 40; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        return -1;
    }

    printf("New termios structure set\n");

    return fd;
}

int disconnectFD(int fd)
{
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        return -1;
    }
    return 0;
}

void alarmHandler(int signal)
{
    alarmCount++;
    alarmEnabled = TRUE;
    printf("Alarm count: %d\n", alarmCount);
}

void alarmDisable()
{
    alarm(0);
    alarmEnabled = FALSE;
    alarmCount = 0;
}

int send_packet_command(int fd, unsigned char A, unsigned char C)
{
    // SET command
    unsigned char buf[FRAME_SIZE] = {FLAG, A, C, 0, FLAG};
    buf[3] = buf[1] ^ buf[2];
    
    if(write(fd, buf, FRAME_SIZE) < 0)
    {
        perror("Error write send command");
        return -1;
    }
    return 0;
}

// Check in pratical class --> Add timer of +- 10 seconds to timeout if the packet isn't received
int receivePacket(int fd, unsigned char A_EXPECTED, unsigned char C_EXPECTED) 
{
    enum state enum_state = START;
    
    while (enum_state != STOP)
    {
        //unsigned char buf[BUF_SIZE] = {0};
        unsigned char byte = 0;
        int bytes;
        if((bytes = read(fd, &byte, sizeof(byte))) < 0)
        {
            perror("Error read DISC command");
            return -1;
        }
        if(bytes > 0){
            switch (enum_state)
            {
            case START:
                if(byte == FLAG) enum_state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if(byte == FLAG) continue;
                if(byte == A_EXPECTED) enum_state = A_RCV;
                else enum_state = START;
                break;
            case A_RCV:
                if(byte == C_EXPECTED) enum_state = C_RCV;
                else if(byte == FLAG) enum_state = FLAG_RCV;
                else enum_state = START;
                break;
            case C_RCV:
                if(byte == (C_EXPECTED ^ A_EXPECTED)) enum_state = BCC_OK;
                else if(byte == FLAG) enum_state = FLAG_RCV;
                else enum_state = START;
                break;
            case BCC_OK:
                if(byte == FLAG) enum_state = STOP;
                else enum_state = START;
                break;      
            default:
                enum_state = START;
            }
        }
    }
    
    return 0; 
}

int receivePacketRetransmission(int fd, unsigned char A_EXPECTED, unsigned char C_EXPECTED, unsigned char A_TO_SEND, unsigned char C_TO_SEND)
{
    enum state enum_state = START;
    (void)signal(SIGALRM, alarmHandler);
    alarm(connectionParameters.timeout); 
    if(send_packet_command(fd, A_TO_SEND, C_TO_SEND)) return -1;

    while (enum_state != STOP && alarmCount <= connectionParameters.nRetransmissions)
    {
        //unsigned char buf[BUF_SIZE] = {0};
        unsigned char byte = 0;
        int bytes;
        if((bytes = read(fd, &byte, sizeof(byte))) < 0)
        {
            perror("Error read UA command");
            return -1;
        }
        if(bytes > 0){
            switch (enum_state)
            {
            case START:
                if(byte == FLAG) enum_state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if(byte == FLAG) continue;
                if(byte == A_EXPECTED) enum_state = A_RCV;
                else enum_state = START;
                break;
            case A_RCV:
                if(byte == C_EXPECTED) enum_state = C_RCV;
                else if(byte == FLAG) enum_state = FLAG_RCV;
                else enum_state = START;
                break;
            case C_RCV:
                if(byte == (C_EXPECTED ^ A_EXPECTED)) enum_state = BCC_OK;
                else if(byte == FLAG) enum_state = FLAG_RCV;
                else enum_state = START;
                break;
            case BCC_OK:
                if(byte == FLAG) enum_state = STOP;
                else enum_state = START;
                break;      
            default:
                enum_state = START;
            }
        }

        if(enum_state == STOP) 
        {
            alarmDisable();
            return 0;
        }
        
        if(alarmEnabled)
        {
            alarmEnabled = FALSE;
            alarm(connectionParameters.timeout);
            if(send_packet_command(fd, A_TO_SEND, C_TO_SEND)) return -1;
            enum_state = START;
        }
    }
    
    return -1;
}

int llopen(LinkLayer connectionParametersApp)
{
    memcpy(&connectionParameters, &connectionParametersApp, sizeof(connectionParametersApp));

    fd = connectFD(connectionParameters.serialPort); // get connection to serial ports.
    if(fd < 0) return -1;

    if(connectionParameters.role == LlTx){
        if(receivePacketRetransmission(fd, A_SEND, C_UA, A_SEND, C_SET)) return -1;
        printf("Connection established\n");
    }
    if(connectionParameters.role == LlRx){
        if(receivePacket(fd, A_SEND, C_SET)) return -1;
        if(send_packet_command(fd, A_SEND, C_UA)) return -1;
        sleep(1);
        printf("Connection established\n");
    }
    return fd;
}

void print_answer(const unsigned char *answer, int n)
{
    for(int i = 0; i < n; i++) 
    {
        printf("buf[%d] = 0x%02X\n", i, answer[i]);
    }
}

const unsigned char * byteStuffing(const unsigned char *buf, int bufSize, int *newSize)
{
    unsigned char *result = (unsigned char *) malloc(bufSize * 2 + 1);
    if(result == NULL) return NULL;
    size_t j = 0;

    for(size_t i = 0; i < bufSize; i++, j++){
        if(buf[i] == FLAG){
            result[j++] = ESC;
            result[j] = ESC_FLAG;
        }
        else if (buf[i] == ESC) {
            result[j++] = ESC;
            result[j] = ESC_ESC;
        }
        else
            result[j] = buf[i];
    }

    result = realloc(result, j);

    if (result == NULL) return NULL;
    
    return result;
}

int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO: Verificar lógica da mudança de valor de C_Ns
    // TODO: Retornar número de bytes escritos
    int newSize;
    const unsigned char *newBuf = byteStuffing(buf, bufSize, &newSize);

    if(newBuf == NULL) return -1;

    print_answer(newBuf, newSize);
    printf("Size of buf: %d\n", newSize);

    unsigned char *trama = (unsigned char *) malloc(newSize + 6);
    
    trama[0] = FLAG;
    trama[1] = A_SEND;
    trama[2] = C_Ns == 0 ? 0x00 : 0x40;
    trama[3] = trama[1] ^ trama[2];
    memcpy(trama + 4, newBuf, newSize);
    unsigned char bcc2 = 0x00;
    for(size_t i = 0; i < newSize; i++) bcc2 ^=  newBuf[i];
    trama[newSize + 4] = bcc2;
    trama[newSize + 5] = FLAG;
    

    enum state enum_state = START;
    (void)signal(SIGALRM, alarmHandler);
    alarm(connectionParameters.timeout); 
    if(write(fd, trama, (newSize + 6)) < 0)
    {
        perror("Error write send command");
        return -1;
    }
    unsigned char C_Nr = 0; // Nr

    while (enum_state != STOP && alarmCount <= connectionParameters.nRetransmissions)
    {
        unsigned char byte = 0;
        int bytes;
        if((bytes = read(fd, &byte, sizeof(byte))) < 0)
        {
            perror("Error read UA command");
            return -1;
        }
        if(bytes > 0){
            switch (enum_state)
            {
            case START:
                if(byte == FLAG) enum_state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if(byte == FLAG) continue;
                if(byte == A_SEND) enum_state = A_RCV;
                else enum_state = START;
                break;
            case A_RCV:
                if(byte == RR0 || byte == RR1 || byte == REJ0 || byte == REJ1) {
                    enum_state = C_RCV;
                    C_Nr = byte;
                }
                else if(byte == FLAG) enum_state = FLAG_RCV;
                else enum_state = START;
                break;
            case C_RCV:
                if(byte == (C_Nr ^ A_SEND)) enum_state = BCC_OK;
                else if(byte == FLAG) enum_state = FLAG_RCV;
                else enum_state = START;
                break;
            case BCC_OK:
                if(byte == FLAG) enum_state = STOP;
                else enum_state = START;
                break;
            default:
                enum_state = START;
            }
        }

        if(enum_state == STOP) 
        {
            if(C_Nr == REJ0 || C_Nr == REJ1){ // timerCounter-- dá-se reset ao alarm?
                alarmEnabled = TRUE;
            }
            if(C_Nr == RR0 || C_Nr == RR1) {
                alarmDisable();
                C_Ns = (C_Ns) ? 0 : 1;
                //C_Ns = C_Nr;
                return newSize + 6; // CHECK THIS OR newSize 
            }
        }
        
        if(alarmEnabled)
        {
            alarmEnabled = FALSE;
            alarm(connectionParameters.timeout);
            if(write(fd, trama, (newSize + 6)) < 0)
            {
                perror("Error write send command");
                return -1;
            }
            enum_state = START;
        }
    }

    return -1;
}

int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

int llclose(int showStatistics)
{
    if(connectionParameters.role == LlTx)
    {
        if(receivePacketRetransmission(fd, A_RECV, C_DISC, A_SEND, C_DISC)) return -1;
        if(send_packet_command(fd, A_SEND, C_UA)) return -1;
        printf("Disconnected\n");
    }
    if(connectionParameters.role == LlRx)
    {
        if(receivePacket(fd, A_SEND, C_DISC)) return -1;
        if(receivePacketRetransmission(fd, A_SEND, C_UA, A_RECV, C_DISC)) return -1;
        printf("Disconnected\n");
    }

    if(disconnectFD(fd)) return -1;
    return 0;

    return 1;
}

