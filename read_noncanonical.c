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

#define FRAME_SIZE 5

#define FLAG 0x7E
#define SS_AR 0x03
#define SR_AS 0x01
#define SET 0x03
#define UA 0x07

volatile int STOP = FALSE;

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
    unsigned char frame[FRAME_SIZE + 1] = {0}; // +1: Save space for the final '\0' char
    unsigned char buf[1];
    int second_flag = FALSE; // Check if second flag is the next to be read
    int idx = 0;

    while (STOP == FALSE)
    {
        // Returns after 5 chars have been input
        int bytes = read(fd, buf, 1);

        if (bytes == -1) {
            perror("read");
            exit(-1);
        }

        frame[idx++] = buf[0]; // Set end of string to '\0', so we can printf

        if (buf[0] == FLAG) {
            if (second_flag) {
                STOP = TRUE;
            }
            else {
                second_flag = TRUE;
            }
        }
    }

    frame[5] = '\0';

    printf("var = 0x%02X\n", frame[0]);
    printf("var = 0x%02X\n", frame[1]);
    printf("var = 0x%02X\n", frame[2]);
    printf("var = 0x%02X\n", frame[3]);
    printf("var = 0x%02X\n", frame[4]);

    unsigned char bcc = frame[1] ^ frame[2];

    printf("BCC = 0x%02X\n", bcc);

    if (bcc == frame[3]) {
        // Everything good!
        unsigned char send_frame[5] = {FLAG, SS_AR, UA, SS_AR ^ UA, FLAG};

        int bytes = write(fd, send_frame, FRAME_SIZE);

        printf("Wrote %d bytes\n", bytes);

        // Wait until all bytes have been written to the serial port
        sleep(1);
    }
    else {
        // Error detected in the frame;
    }

    // The while() cycle should be changed in order to respect the specifications
    // of the protocol indicated in the Lab guide

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
