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

#define BUF_SIZE 5

#define FLAG    0x7E
#define A_SEND  0x03
#define A_RECV  0x01
#define C_SET   0x03
#define C_UA    0x07

volatile int STOP = FALSE;

int check_set(unsigned char *answer){
    if(answer[0] == FLAG &&
        answer[1] == A_SEND && 
        answer[2] == C_SET &&
        answer[3] == (A_SEND ^ C_SET) && 
        answer[4] == FLAG)
    return TRUE;
    return FALSE;
}

void print_answer(unsigned char *answer){
    printf("flag =  0x%02X\n", answer[0]);
    printf("a =     0x%02X\n", answer[1]);
    printf("c =     0x%02X\n", answer[2]);
    printf("xor ac= 0x%02X\n", answer[3]);
    printf("flag =  0x%02X\n", answer[4]);
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
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 5;  // Blocking read until 5 chars received

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

    while (STOP == FALSE)
    {
        // Returns after 5 chars have been input
        if(read(fd, buf, BUF_SIZE) < 0){
            perror("Error read SET command");
            exit(-1);
        }
        if(check_set(buf)) STOP = TRUE;
    }


    buf[0] = FLAG;
    buf[1] = A_RECV;
    buf[2] = C_UA;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
        
    int bytes; 
    if((bytes = write(fd, buf, BUF_SIZE)) < 0){
        perror("Error write UA command");
        exit(-1);
    }
    printf("%d bytes written\n", bytes);

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

