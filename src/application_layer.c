// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define DATA 1
#define C_START 2
#define C_END 3

#define T_FILESIZE 0
#define T_FILENAME 1
#define MAX_PAYLOAD_SIZE 1000

#define MAX_FILENAME 50

enum state{
    RECV_START,
    RECV_CONT,
    RECV_END
};

enum state stateReceive = RECV_START;

int sendPacketData(size_t nBytes, unsigned char *data) 
{
    if(data == NULL) return -1;

    unsigned int L2 = nBytes / 256, L1 = nBytes % 256;
    
    unsigned char *packet = (unsigned char *) malloc(nBytes + 3);
    
    packet[0] = DATA;
    packet[1] = L2;
    packet[2] = L1;
    memcpy(packet + 3, data, nBytes);

    return llwrite(packet, nBytes + 3);
}

unsigned char * itouchar(size_t file_size, unsigned char *size)
{
    if (size == NULL) return NULL; 
    
    unsigned char * bytes = malloc(100); 
    if (bytes == NULL) return NULL; 
    
    size_t indx = 0;

    if(file_size == 0) {
        bytes[indx++] = 0;
    } else {
        while (file_size != 0) {
            bytes[indx++] = file_size % 256; 
            file_size = file_size / 256;
        }
    }

    bytes = realloc(bytes, indx);

    *size = (int) indx;
    return bytes;
}

size_t uchartoi (unsigned char n, unsigned char * numbers)
{
    if(numbers == NULL) return 0; // TODO: check later
    int value = 0;
    int power = 1;
    for(int i = 0; i < n; i++, power *= 256){
        value += numbers[i] * power;
    }
    return (size_t) value;
}

// C TLV TLV ; file_size = L0 * 256^0 + L1 * 256^1 + L2 * 256^2....
int sendPacketControl(unsigned char C, const char * filename, size_t file_size)
{
    if(filename == NULL) return -1;
    
    unsigned char L1 = 0;
    unsigned char * V1 = itouchar(file_size, &L1);
    if(V1 == NULL) return -1;

    unsigned char L2 = (unsigned char) strlen(filename);
    unsigned char *packet = (unsigned char *) malloc(5 + L1 + L2);
    if(packet == NULL) return -1;

    size_t indx = 0;
    packet[indx++] = C;
    packet[indx++] = T_FILESIZE;
    packet[indx++] = L1;
    memcpy(packet + indx, V1, L1); indx += L1;
    packet[indx++] = T_FILENAME;
    packet[indx++] = L2;
    memcpy(packet + indx, filename, L2); indx += L2;  // check later. for cycle if it doesnt work

    free(V1);
    
    int res = llwrite(packet, (int) indx);

    free(packet);

    return res;
}

unsigned char * readPacketData(unsigned char *buff, size_t *newSize)
{
    if (buff == NULL) return NULL;
    if (buff[0] != DATA) return NULL;

    *newSize = buff[1] * 256 + buff[2];

    return buff + 3;
}

// Ciclo for com numero de TLVs, switch para saber que TLV é que é, retornar erro se T > 1 ou T < 0 (?)
int readPacketControl(unsigned char * buff, size_t * file_size, char * file_name)
{   
    if (buff == NULL || file_size == NULL || file_name == NULL) return -1;
    size_t indx = 0;

    if(buff[indx] == C_START) stateReceive = RECV_CONT;
    else if(buff[indx] == C_END) stateReceive = RECV_END;
    else return -1;

    indx++;
    if (buff[indx++] != T_FILESIZE) return -1;
    unsigned char L1 = buff[indx++];
    unsigned char * V1 = malloc(L1);
    if(V1 == NULL) return -1;
    memcpy(V1, buff + indx, L1); indx += L1;
    *file_size = uchartoi(L1, V1);
    free(V1);

    if(buff[indx++] != T_FILENAME) return -1;
    unsigned char L2 = buff[indx++];
    memcpy(file_name, buff + indx, L2);
    file_name[L2] = '\0';

    return 0;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    if(serialPort == NULL || role == NULL || filename == NULL) printf("some arguments are null\n");
    LinkLayer connectionParametersApp;
    strncpy(connectionParametersApp.serialPort, serialPort, sizeof(connectionParametersApp.serialPort)-1);
    connectionParametersApp.role = strcmp(role, "tx") ? LlRx : LlTx; 
    connectionParametersApp.baudRate = baudRate;
    connectionParametersApp.nRetransmissions = nTries;
    connectionParametersApp.timeout = timeout;

    if (llopen(connectionParametersApp) == -1) {
        perror("Error in llopen'\n");
        return;
    }
    

    if (connectionParametersApp.role == LlTx) {

        size_t bytesRead = 0;
        unsigned char *buffer = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
        if(buffer == NULL) printf("167 error\n");
        FILE* file = fopen(filename, "rb");

        fseek(file, 0, SEEK_END);
        size_t file_size = ftell(file);
        rewind(file);

        if(sendPacketControl(C_START, filename, file_size) == -1) printf("170 error\n");

        while ((bytesRead = fread(buffer, 1, MAX_PAYLOAD_SIZE, file)) > 0) {
            size_t sended_bytes = 0;
            sended_bytes = sendPacketData(bytesRead, buffer);
            if(sended_bytes == -1) printf("error\n");

            printf("line 172: %lu\n", sended_bytes);
            if(bytesRead < MAX_PAYLOAD_SIZE){
                if(sendPacketControl(C_END, filename, file_size) == -1) printf("179 error\n");
                fclose(file);
                break;
            }
        }
    } else {
        char * file_name = malloc(MAX_FILENAME);
        unsigned char * buf = malloc(MAX_PAYLOAD_SIZE + 200); // TODO: check
        unsigned char * packet = malloc(MAX_PAYLOAD_SIZE + 100);
        if(file_name == NULL || buf == NULL || packet == NULL) printf("error\n");

        size_t file_size = 0;
        FILE *file = fopen(filename, "w");
        size_t bytes_readed = 0;
        
        while(stateReceive != RECV_END){
            bytes_readed = llread(buf);
            if(bytes_readed == -1) printf("197 error\n");
            
            if(buf[0] == C_START || buf[0] == C_END){
                if(readPacketControl(buf, &file_size, file_name) == -1) printf("200 error\n");
            }else if(buf[0] == DATA){

                packet = readPacketData(buf, &bytes_readed);
                if(packet == NULL) printf("203 error\n");

                fwrite(packet, 1, bytes_readed, file);
                
            }
        }
        fclose(file);
    }


    if (llclose(1) == -1) {
        perror("Error in llclose'\n");
        return;
    }
}
