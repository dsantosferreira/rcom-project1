// Write to serial port in non-canonical mode

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

enum state_ua{
    START, 
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
};

int horaCertaIrmao = FALSE;
int alarmCount = 0;
enum state_ua enum_state_ua = START;

void alarmHandler(int signal)
{
    alarmCount++;
    horaCertaIrmao = TRUE;
    printf("Alarm count: %d\n", alarmCount);
}

void alarmDisable()
{
    alarm(0);
}

void print_answer(unsigned char *answer){
    printf("flag =  0x%02X\n", answer[0]);
    printf("a =     0x%02X\n", answer[1]);
    printf("c =     0x%02X\n", answer[2]);
    printf("xor ac= 0x%02X\n", answer[3]);
    printf("flag =  0x%02X\n", answer[4]);
}

void send_SET_command(int fd)
{
    // SET command
    unsigned char buf[FRAME_SIZE] = {FLAG, A_SEND, C_SET, 0, FLAG};
    buf[3] = buf[1] ^ buf[2];
    
    int bytes;
    if(write(fd, buf, FRAME_SIZE) < 0)
    {
        perror("Error write send command");
        exit(-1);
    }
}

int main(int argc, char *argv[]){

    const char *serialPortName = argv[1];

    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS1\n",
               argv[0],
               argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 30; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 5 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");


    (void)signal(SIGALRM, alarmHandler);
    alarm(3); // set alarm to 3 seconds.

    send_SET_command(fd);

    unsigned char A, C;
    while (enum_state_ua != STOP && alarmCount < MAX_RET_ATTEMPTS)
    {
        unsigned char buf[BUF_SIZE] = {0};
        int bytes;
        // printf("line 147\n");
        if((bytes = read(fd, buf, BUF_SIZE)) < 0)
        {
            perror("Error read UA command");
            exit(-1);
        }
        if(bytes > 0){
            switch (enum_state_ua)
            {
            case START:
                if(buf[0] == FLAG) enum_state_ua = FLAG_RCV;
                break;
            case FLAG_RCV:
                if(buf[0] == FLAG) continue;
                if(buf[0] == A_SEND){
                    A = buf[0];
                    enum_state_ua = A_RCV;
                }
                else enum_state_ua = START;
                break;
            case A_RCV:
                if(buf[0] == C_UA) {
                    enum_state_ua = C_RCV;
                    C = buf[0];
                }
                else if(buf[0] == FLAG) enum_state_ua = FLAG_RCV;
                else enum_state_ua = START;
                break;
            case C_RCV:
                if(buf[0] == (C ^ A)) enum_state_ua = BCC_OK;
                else if(buf[0] == FLAG) enum_state_ua = FLAG_RCV;
                else enum_state_ua = START;
                break;
            case BCC_OK:
                if(buf[0] == FLAG) enum_state_ua = STOP;
                else enum_state_ua = START;
                break;      
            default:
                enum_state_ua = START;
            }
        }


        if(enum_state_ua == STOP)
        {
            printf("Connection established\n");
            alarmDisable();
        }

        if(horaCertaIrmao)
        {
            horaCertaIrmao = FALSE;
            alarm(3);
            send_SET_command(fd);
            enum_state_ua = START;
        }
    }
    

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
