#include "data_link.h"


int alarmEnabled = FALSE;
int alarmCount = 0;
struct termios oldtio;


void alarmHandler(int signal)
{
    alarmCount++;
    alarmEnabled = TRUE;
    printf("Alarm count: %d\n", alarmCount);
}

void alarmDisable()
{
    alarm(0);
}

void print_answer(unsigned char *answer, int n)
{
    for(int i = 0; i < n; i++) 
    {
        printf("answer[%d] = 0x%02X\n", i, answer[i]);
    }
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

int llopen(const char * port, unsigned char flag)
{
    int fd = connectFD(port); // get connection to serial ports.
    if(fd < 0) return -1;

    enum state enum_state = START;

    if(flag == TRANSMITTER){

        (void)signal(SIGALRM, alarmHandler);
        alarm(3); // set alarm to 3 seconds.

        if(send_packet_command(fd, A_SEND, C_SET)) return -1;

        while (enum_state != STOP && alarmCount < MAX_RET_ATTEMPTS)
        {
            //unsigned char buf[BUF_SIZE] = {0};
            unsigned char byte = 0;
            int bytes;
            // printf("line 147\n");
            if((bytes = read(fd, &byte, sizeof(byte))) < 0)
            {
                perror("Error read UA command");
                exit(-1);
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
                    if(byte == C_UA) enum_state = C_RCV;
                    else if(byte == FLAG) enum_state = FLAG_RCV;
                    else enum_state = START;
                    break;
                case C_RCV:
                    if(byte == (C_UA ^ A_SEND)) enum_state = BCC_OK;
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
                printf("Connection established\n");
                alarmDisable();
            }

            if(alarmEnabled)
            {
                alarmEnabled = FALSE;
                alarm(3);
                if(send_packet_command(fd, A_SEND, C_SET)) return -1;
                enum_state = START;
            }
        }
    }
    if(flag == RECEIVER){
        //unsigned char buf[BUF_SIZE] = {0};
        unsigned char byte = 0;
        int bytes;

        while (enum_state != STOP)
        {
            if((bytes = read(fd, &byte, sizeof(byte))) < 0){ 
                perror("Error read SET command");
                exit(-1);
            }
            //printf("Was readed: %hhx\n", byte);
            if(bytes > 0){ // Uncesssary with time = 0 and min > 0
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
                    if(byte == C_SET) enum_state = C_RCV;
                    else if(byte == FLAG) enum_state = FLAG_RCV;
                    else enum_state = START;
                    break;
                case C_RCV:
                    if(byte == (C_SET ^ A_SEND)) enum_state = BCC_OK; 
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

        if(send_packet_command(fd, A_SEND, C_UA)) return -1;

        // Wait until all bytes have been written to the serial port
        sleep(1);
        printf("Connection established\n");
    }
    return fd;
}

int llclose(int fd, unsigned char flag) // temos que fazer aqui a retrnsmission?
{
    enum state enum_state = START;
    if(flag == TRANSMITTER)
    {
        (void)signal(SIGALRM, alarmHandler);
        alarm(3); // set alarm to 3 seconds.

        if(send_packet_command(fd, A_SEND, C_DISC)) return -1;

        while (enum_state != STOP && alarmCount < MAX_RET_ATTEMPTS)
        {
            //unsigned char buf[BUF_SIZE] = {0};
            unsigned char byte = 0;
            int bytes;
            if((bytes = read(fd, &byte, sizeof(byte))) < 0)
            {
                perror("Error read DISC command");
                exit(-1);
            }
            if(bytes > 0){
                switch (enum_state)
                {
                case START:
                    if(byte == FLAG) enum_state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if(byte == FLAG) continue;
                    if(byte == A_RECV) enum_state = A_RCV;
                    else enum_state = START;
                    break;
                case A_RCV:
                    if(byte == C_DISC) enum_state = C_RCV;
                    else if(byte == FLAG) enum_state = FLAG_RCV;
                    else enum_state = START;
                    break;
                case C_RCV:
                    if(byte == (C_DISC ^ A_RECV)) enum_state = BCC_OK;
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

            if(enum_state == STOP) alarmDisable();
            
            if(alarmEnabled)
            {
                alarmEnabled = FALSE;
                alarm(3);
                if(send_packet_command(fd, A_SEND, C_DISC)) return -1;
                enum_state = START;
            }
        }
        if(send_packet_command(fd, A_SEND, C_UA)) return -1;
    }
    if(flag == RECEIVER)
    {
        while (enum_state != STOP)
        {
            //unsigned char buf[BUF_SIZE] = {0};
            unsigned char byte = 0;
            int bytes;
            if((bytes = read(fd, &byte, sizeof(byte))) < 0)
            {
                perror("Error read DISC command");
                exit(-1);
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
                    if(byte == C_DISC) enum_state = C_RCV;
                    else if(byte == FLAG) enum_state = FLAG_RCV;
                    else enum_state = START;
                    break;
                case C_RCV:
                    if(byte == (C_DISC ^ A_SEND)) enum_state = BCC_OK;
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

        enum_state = START;
        if(send_packet_command(fd, A_RECV, C_DISC)) return -1;

        while (enum_state != STOP)
        {
            //unsigned char buf[BUF_SIZE] = {0};
            unsigned char byte = 0;
            int bytes;
            if((bytes = read(fd, &byte, sizeof(byte))) < 0)
            {
                perror("Error read DISC command");
                exit(-1);
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
                    if(byte == C_UA) enum_state = C_RCV;
                    else if(byte == FLAG) enum_state = FLAG_RCV;
                    else enum_state = START;
                    break;
                case C_RCV:
                    if(byte == (C_UA ^ A_SEND)) enum_state = BCC_OK;
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
    }

    if(disconnectFD(fd)) return -1;
    printf("Disconnected\n");
    return 0;
}

int connectFD(const char * port)
{
    int fd = open(port, O_RDWR | O_NOCTTY);

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

