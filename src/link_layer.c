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

#define C_INF0      0x00
#define C_INF1      0x40

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
    DATA,
    STUFF,
    STOP
};

LinkLayer connectionParameters;
struct termios oldtio;
int alarmEnabled = FALSE;
int alarmCount = 0;
int fd;
unsigned char C_Ns = 0; // Ns
unsigned char C_Nr = 0; // Nr (o valor que ele espera de receber)

int connectFD(const char * port)
{
    if(port == NULL) return -1;
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
    
    //(void)signal(SIGALRM, alarmHandler);
    //alarm(connectionParameters.timeout * connectionParameters.nRetransmissions + 1); 
    while (enum_state != STOP)
    {
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
        // if(alarmEnabled) return -1;
    }
    
    return 0; 
}

int receivePacketRetransmission(int fd, unsigned char A_EXPECTED, unsigned char C_EXPECTED, unsigned char A_TO_SEND, unsigned char C_TO_SEND)
{
    enum state enum_state = START;
    (void)signal(SIGALRM, alarmHandler);
    if(send_packet_command(fd, A_TO_SEND, C_TO_SEND)) return -1;
    alarm(connectionParameters.timeout); 

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
            if(send_packet_command(fd, A_TO_SEND, C_TO_SEND)) return -1;
            alarm(connectionParameters.timeout);
            enum_state = START;
        }
    }
    
    return -1;
}

int llopen(LinkLayer connectionParametersApp)
{
    memcpy(&connectionParameters, &connectionParametersApp, sizeof(connectionParametersApp));

    fd = connectFD(connectionParameters.serialPort);
    if(fd < 0) return -1;

    if(connectionParameters.role == LlTx){
        if(receivePacketRetransmission(fd, A_SEND, C_UA, A_SEND, C_SET)) return -1;
        printf("Connection established\n");
    }
    if(connectionParameters.role == LlRx){
        if(receivePacket(fd, A_SEND, C_SET)) return -1;
        if(send_packet_command(fd, A_SEND, C_UA)) return -1;
        printf("Connection established\n");
    }
    return 1; // TODO: Return fd?
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
    if(buf == NULL || newSize == NULL) return NULL;

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

    *newSize = (int) j;
    result = realloc(result, j);

    if (result == NULL) return NULL;

    return result;
}

int llwrite(const unsigned char *buf, int bufSize)
{

    // TODO: Remove this later

    int kaboom = TRUE;

    // TODO: Verificar lógica da mudança de valor de C_Ns
    // TODO: Retornar número de bytes escritos
    // TODO: Temos que ter newBuf, nao podemos usar so buf? ou free(buf)? Calcular bcc2 logo no inicio.
    if(buf == NULL) return -1;
    int newSize;
    const unsigned char *newBuf = byteStuffing(buf, bufSize, &newSize);
    if(newBuf == NULL) return -1;
    printf("new Size de write%d\n", newSize);
    unsigned char *trama = (unsigned char *) malloc(newSize + 6);
    if(trama == NULL) return -1;

    trama[0] = FLAG;
    trama[1] = A_SEND;
    trama[2] = (C_Ns) ? C_INF1 : C_INF0;
    trama[3] = trama[1] ^ trama[2];
    memcpy(trama + 4, newBuf, newSize);

    for(int i = 0; i < 5; i++) 
    {
        printf("buf[%d] = 0x%02X\n", i, trama[i]);
    }

    unsigned char bcc2 = 0x00;
    for(size_t i = 0; i < bufSize; i++) bcc2 ^=  buf[i];
    trama[newSize + 4] = bcc2; 
    if(bcc2 == FLAG){
        trama[newSize + 4] = ESC;
        newSize++;
        trama[newSize + 4] = ESC_FLAG;
        trama = realloc(trama, newSize + 6);
        trama[newSize + 5] = FLAG;
    }else{
        trama[newSize + 5] = FLAG;
    }
    

    enum state enum_state = START;
    (void)signal(SIGALRM, alarmHandler);
    if(write(fd, trama, (newSize + 6)) < 0)
    {
        perror("Error write send command");
        return -1;
    }
    alarm(connectionParameters.timeout); 
    unsigned char C_received = 0, A_received = 0;

    while (enum_state != STOP && alarmCount <= connectionParameters.nRetransmissions)
    {
        //printf("state: %d\n", enum_state);
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
                C_received = 0;
                A_received = 0;
                if(byte == FLAG) enum_state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if(byte == FLAG) continue;
                if(byte == A_SEND || byte == A_RECV) {
                    enum_state = A_RCV;
                    A_received = byte;
                }
                else enum_state = START;
                break;
            case A_RCV:
                if(byte == RR0 || byte == RR1 || byte == REJ0 || byte == REJ1) {
                    enum_state = C_RCV;
                    C_received = byte;
                }
                else if(byte == FLAG) enum_state = FLAG_RCV;
                else enum_state = START;
                break;
            case C_RCV:
                if(byte == (C_received ^ A_received)) enum_state = BCC_OK;
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
            if(C_received == REJ0 || C_received == REJ1){ // timerCounter-- dá-se reset ao alarm?
                alarmEnabled = TRUE;
                //printf("received reject\n");
                if (kaboom) {
                    print_answer(trama, 106);
                    kaboom = FALSE;
                }
            }
            if(C_received == RR0 || C_received == RR1) {
                // alarmDisable(); Vai comecar fazer alarmCount de 0. or alarm(0)?
                C_Ns = 1 - C_Ns;
                //C_Ns = C_received;
                return bufSize; // CHECK THIS OR newSize + 6? 
            }
        }
        
        if(alarmEnabled)
        {
            alarmEnabled = FALSE;
            if(write(fd, trama, (newSize + 6)) < 0)
            {
                perror("Error write send command");
                return -1;
            }
            alarm(connectionParameters.timeout);
            enum_state = START;
        }
    }

    return -1;
}

unsigned char * byteDestuffing(unsigned char *buf, int bufSize, int *newSize, unsigned char *bcc2_received)
{
    if(buf == NULL || newSize == NULL) return NULL;

    unsigned char *result = (unsigned char *) malloc(bufSize);
    if(result == NULL) return NULL;
    size_t j = 0; // index of result array

    for(size_t i = 0; i < bufSize; i++){
        if(buf[i] != ESC) result[j++] = buf[i]; 
        else{
            if(i + 1 == bufSize){ // caso se ultimo byte era ESC
                result[j++] = buf[i]; 
                break;
            }
            if(buf[i + 1] == ESC_FLAG) {result[j++] = FLAG; i++;}
            else if(buf[i + 1] == ESC_ESC) {result[j++] = ESC; i++;}
        }
    }
    
    *bcc2_received = result[j - 1];
    *newSize = (int) (j - 1); // newSize include '\0' value
    result = realloc(result, j - 1);
    if (result == NULL) return NULL;

    return result;
}

int llread(unsigned char *packet)
{
    enum state enum_state = START;
    unsigned char C_received = 0; // o valor C recebido de trama.
    size_t pkt_indx = 0; // o index atual de escrita na packet.
    while (enum_state != STOP)
    {
        printf("state: %d\n", enum_state);
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
                C_received = 0;
                pkt_indx = 0;
                if(byte == FLAG) enum_state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if(byte == FLAG) continue;
                if(byte == A_SEND) enum_state = A_RCV;
                else enum_state = START;
                break;
            case A_RCV:
                printf("byte = 0x%02X\n", byte);
                if(byte == C_INF0 || byte == C_INF1){
                    enum_state = C_RCV;
                    C_received = byte;
                }
                else if(byte == FLAG) enum_state = FLAG_RCV;
                else enum_state = START;
                break;
            case C_RCV:
                if(byte == (C_received ^ A_SEND)) enum_state = DATA; // need to check A_SEND
                else if(byte == FLAG) enum_state = FLAG_RCV;
                else enum_state = START;
                break;
            case DATA: // se for FLAG passa para BCC_OK
                if(byte == FLAG){
                    printf("byte == FLAG\n");
                    enum_state = STOP;
                    int newSize = 0;
                    unsigned char bcc2_received;
                    
                    packet = byteDestuffing(packet, pkt_indx, &newSize, &bcc2_received);
                    printf("pkt_indx = %ld\n", pkt_indx);
                    printf("newSize = %d\n", newSize);
                    unsigned char bcc2 = 0x00;
                    for(size_t i = 0; i < newSize; i++) bcc2 ^= packet[i];

                    unsigned char C_respons, A_respons;
                    // Check this!!!
                    if(bcc2 == bcc2_received) {
                        C_respons = (C_received == C_INF0)? RR1 : RR0;
                        A_respons = A_SEND;
                    }
                    else{
                        C_respons = (C_received == C_INF0)? REJ0 : REJ1;
                        A_respons = A_RECV;
                    } 

                    if(send_packet_command(fd, A_respons, C_respons)) return -1;
                    if((C_Nr == 0 && C_received == C_INF0) || (C_Nr == 1 && C_received == C_INF1)){
                        C_Nr = 1 - C_Nr;
                        return newSize;
                    } 
                    enum_state = START;
                }
                packet[pkt_indx++] = byte;
                break;
            default:
                enum_state = START;
            }
        }
    }
    return -1;
}

// Do we need here roles?
int llclose(int showStatistics)
{
    // TODO: show Statistics!
    printf("DISCONNECT\n");
    if(connectionParameters.role == LlTx)
    {
        if(receivePacketRetransmission(fd, A_SEND, C_DISC, A_SEND, C_DISC)) return -1;
        if(send_packet_command(fd, A_SEND, C_UA)) return -1;
        printf("Disconnected\n");
    }
    if(connectionParameters.role == LlRx)
    {
        if(receivePacket(fd, A_SEND, C_DISC)) return -1;
        if(receivePacketRetransmission(fd, A_SEND, C_UA, A_SEND, C_DISC)) return -1;
        printf("Disconnected\n");
    }

    if(disconnectFD(fd)) return -1;
    return 0;

    return 1;
}

