// Read from serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

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

enum state_set{
    START, 
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP
};

enum state_set enum_state_set = START;

void print_answer(unsigned char *answer, int n){
    for(int i = 0; i < n; i++) {
        printf("answer[%d] = 0x%02X\n", i, answer[i]);
    }
}

int main(int argc, char *argv[])
{
    // Program usage: Uses either COM1 or COM2
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

    // Open serial port device for reading and writing and not as controlling tty
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
    memset(&newtio, 0, sizeof(newtio)); // is memset_s better?

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 1;  // Blocking read until 5 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    // Loop for input
    unsigned char buf[BUF_SIZE] = {0};
    unsigned char A, C;
    int bytes;

    while (enum_state_set != STOP)
    {
        if((bytes = read(fd, buf, BUF_SIZE)) < 0){ 
            perror("Error read SET command");
            exit(-1);
        }
        if(bytes > 0){ // Uncesssary with time = 0 and min > 0
            switch (enum_state_set)
            {
            case START:
                if(buf[0] == FLAG) enum_state_set = FLAG_RCV;
                break;
            case FLAG_RCV:
                if(buf[0] == FLAG) continue;
                if(buf[0] == A_SEND){
                    A = buf[0];
                    enum_state_set = A_RCV;
                }
                else enum_state_set = START;
                break;
            case A_RCV:
                if(buf[0] == C_SET) {
                    enum_state_set = C_RCV;
                    C = buf[0];
                }
                else if(buf[0] == FLAG) enum_state_set = FLAG_RCV;
                else enum_state_set = START;
                break;
            case C_RCV:
                if(buf[0] == (C ^ A)) enum_state_set = BCC_OK; 
                else if(buf[0] == FLAG) enum_state_set = FLAG_RCV;
                else enum_state_set = START;
                break;
            case BCC_OK:
                if(buf[0] == FLAG) enum_state_set = STOP;
                else enum_state_set = START;
                break;      
            default:
                enum_state_set = START;
            }
        }
    }

    unsigned char frame_buf[FRAME_SIZE] = {0};
    frame_buf[0] = FLAG;
    frame_buf[1] = A_SEND;
    frame_buf[2] = C_UA;
    frame_buf[3] = frame_buf[1] ^ frame_buf[2];
    frame_buf[4] = FLAG;
        
    if(write(fd, frame_buf, FRAME_SIZE) < 0){
        perror("Error write UA command");
        exit(-1);
    }

    // Wait until all bytes have been written to the serial port
    sleep(1);
    printf("Connection established\n");

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
