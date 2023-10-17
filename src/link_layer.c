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
#include <sys/time.h>

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

typedef struct
{
    size_t bytes_readed;
    unsigned int nFrames;       // Number of good frames sent/received
    unsigned int frames_size;   // Size of good frames sent
    double time_send_control;   // Time spent on sending control frames
    double time_send_data;      // Time spent on sending data frames
    struct timeval start;       // When program starts
} Statistics;

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
Statistics statistics = {0, 0, 0, 0.0, 0.0};
struct termios oldtio;
int alarmEnabled = FALSE;
int alarmCount = 0;
int fd;
unsigned char C_Ns = 0; // Ns
unsigned char C_Nr = 0; // Nr (o valor que ele espera de receber)

int connectFD(LinkLayer connectionParametersApp)
{
    if(connectionParametersApp.serialPort == NULL) return -1;
    fd = open(connectionParametersApp.serialPort, O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        perror(connectionParametersApp.serialPort);
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

    newtio.c_cflag = connectionParametersApp.baudRate | CS8 | CLOCAL | CREAD;
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
    unsigned char buf[FRAME_SIZE] = {FLAG, A, C, 0, FLAG};
    buf[3] = buf[1] ^ buf[2];
    
    if(write(fd, buf, FRAME_SIZE) < 0)
    {
        perror("Error write send command");
        return -1;
    }
    return 0;
}

int receivePacket(int fd, unsigned char A_EXPECTED, unsigned char C_EXPECTED) 
{
    enum state enum_state = START;
    
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
    gettimeofday(&statistics.start, NULL);
    memcpy(&connectionParameters, &connectionParametersApp, sizeof(connectionParametersApp));

    fd = connectFD(connectionParameters);
    if(fd < 0) return -1;

    if(connectionParameters.role == LlTx){
        if(receivePacketRetransmission(fd, A_SEND, C_UA, A_SEND, C_SET)) return -1;
        printf("Connection established\n");
    }
    if(connectionParameters.role == LlRx){
        if(receivePacket(fd, A_SEND, C_SET)) return -1;
        statistics.nFrames++;
        statistics.bytes_readed += 5;
        if(send_packet_command(fd, A_SEND, C_UA)) return -1;
        printf("Connection established\n");
    }
    return 1;
}

void print_answer(const unsigned char *answer, int n)
{
    for(int i = 0; i < n; i++) 
        printf("buf[%d] = 0x%02X\n", i, answer[i]);
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
    // TODO: Verificar lógica da mudança de valor de C_Ns
    if(buf == NULL) return -1;

    int newSize;
    const unsigned char *newBuf = byteStuffing(buf, bufSize, &newSize);
    if(newBuf == NULL) return -1;
    printf("Bytes sended: %d\n", newSize);
    unsigned char *trama = (unsigned char *) malloc(newSize + 6);
    if(trama == NULL) return -1;

    trama[0] = FLAG;
    trama[1] = A_SEND;
    trama[2] = (C_Ns) ? C_INF1 : C_INF0;
    trama[3] = trama[1] ^ trama[2];
    memcpy(trama + 4, newBuf, newSize);


    unsigned char bcc2 = 0x00;
    for(size_t i = 0; i < bufSize; i++) bcc2 ^=  buf[i];
    trama[newSize + 4] = bcc2; 
    if(bcc2 == FLAG){
        printf("BCC2 == FLAG\n");
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
            perror("Error read command");
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
                alarmCount = 0; // Perguntar se isto está certo
                printf("Received reject; Second try.\n");
            }
            if(C_received == RR0 || C_received == RR1) {
                alarmDisable();
                C_Ns = 1 - C_Ns;
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

int byteDestuffing(unsigned char *buf, int bufSize, int *newSize, unsigned char *bcc2_received)
{
    if(buf == NULL || newSize == NULL) return -1;

    unsigned char *result = (unsigned char *) malloc(bufSize);
    if(result == NULL) return -1;
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
    *newSize = (int) (j - 1); 
    result = realloc(result, j - 1);
    if (result == NULL) return -1;
    memcpy(buf, result, j - 1);
    return 0;
}

int llread(unsigned char *packet)
{
    enum state enum_state = START;
    unsigned char C_received = 0; // o valor C recebido de trama.
    size_t pkt_indx = 0; // o index atual de escrita na packet.
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
                    enum_state = STOP;
                    int newSize = 0;
                    unsigned char bcc2_received = 0;
                    
                    if (byteDestuffing(packet, pkt_indx, &newSize, &bcc2_received)) return -1;

                    unsigned char bcc2 = 0x00;
                    for (size_t i = 0; i < newSize; i++) bcc2 ^= packet[i];

                    unsigned char C_respons, A_respons;

                    if (bcc2 == bcc2_received) {
                        C_respons = (C_received == C_INF0)? RR1 : RR0;
                        A_respons = A_SEND;
                    }
                    else {
                        C_respons = (C_received == C_INF0)? REJ0 : REJ1;
                        A_respons = A_RECV;
                    } 

                    if (send_packet_command(fd, A_respons, C_respons)) return -1;

                    enum_state = START;

                    if (C_respons == REJ0 || C_respons == REJ1) break;

                    if ((C_Nr == 0 && C_received == C_INF0) || (C_Nr == 1 && C_received == C_INF1)){
                        C_Nr = 1 - C_Nr;
                        printf("Bytes received: %d\n", newSize);
                        statistics.bytes_readed += newSize;
                        statistics.nFrames++;
                        return newSize;
                    } 
                    printf("Received duplicate\n");
                } else packet[pkt_indx++] = byte;
                
                break;
            default:
                enum_state = START;
            }
        }
    }
    return -1;
}

void printStatistics()
{
    printf("\n======== Statistics ========\n");
    // TODO: se for receiver ele nao tem acceso ao sended bytes.
    // TODO: adicioanr mais estatistica.

    // TODO: Número de bytes totais recebidos pelo recetor (antes de destuffing)

    // Número de bytes totais recebidos pelo recetor (após destuffing)

    // Time taken to download file
    struct timeval end;
    gettimeofday(&end, NULL);
    double time_spent = (end.tv_sec - statistics.start.tv_sec) + 
                        (end.tv_usec - statistics.start.tv_usec) / 1e6;

    // TODO: Número de frames recebidas. Só no recetor
    if (connectionParameters.role == LlRx) {
        // Número de bytes totais recebidos pelo recetor (após destuffing)
        printf("\n============================\n");
        printf("Number of bytes received (after destuffing): %lu\n", statistics.bytes_readed);
        printf("============================\n");

        // Número de frames recebidos
        printf("\n============================\n");
        printf("Number of good frames received: %d frames\n", statistics.nFrames);
        printf("============================\n");

        // Tamanho necessário para transferir o ficheiro completo
        printf("\n============================\n");
        printf("Time taken to download file: %f seconds\n", time_spent);
        printf("============================\n");

        // TODO: Tamanho médio das frames só no recetor
    }

    else {
        // TODO: Tempo que se passa a mandar frames de controlo só no transmissor


        // TODO: Tempo que se passa a mandar frames de dados só no transmissor


        // TODO: Tempo médio que demora a mandar uma frame só no transmissor
    }
}

int llclose(int showStatistics)
{
    if(connectionParameters.role == LlTx)
    {
        if(receivePacketRetransmission(fd, A_SEND, C_DISC, A_SEND, C_DISC)) return -1;
        if(send_packet_command(fd, A_SEND, C_UA)) return -1;
        printf("Disconnected\n");
    }
    if(connectionParameters.role == LlRx)
    {
        if(receivePacket(fd, A_SEND, C_DISC)) return -1;
        statistics.nFrames++;
        statistics.bytes_readed += 5;

        // Mudar para send_packet_command como o stor tinha dito na aula?
        if(receivePacketRetransmission(fd, A_SEND, C_UA, A_SEND, C_DISC)) return -1;
        statistics.nFrames++;           // Retirar este linha se se mudar em cima para send_packet_command
        statistics.bytes_readed += 5;   // Retirar este linha se se mudar em cima para send_packet_command

        printf("Disconnected\n");
    }

    if(disconnectFD(fd)) return -1;
    if(showStatistics) printStatistics();
    return 1;
}

