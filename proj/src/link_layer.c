// Link layer protocol implementation

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include "link_layer.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

LinkLayer connection;
struct termios oldtio;
struct termios newtio;
int fd;

#define FLAG (0b01111110)
#define ESCAPE (0x7d)
#define ESCAPE_FLAG (0x5e)
#define ESCAPE_ESCAPE (0x5d)
#define ADR_TX (0b00000011)
#define ADR_RX (0b00000001)
#define CTRL_SET (0b00000011)
#define CTRL_DISC (0b00001011)
#define CTRL_UA (0b00000111)
#define CTRL_RR(R) (R%2?0b10000101:0b00000101)
#define CTRL_REJ(R) (R%2?0b10000001:0b00000001)
#define CTRL_DATA(S) (S%2?0b01000000:0b00000000)
#define PACKET_SIZE_LIMIT 256
unsigned char alarmEnabled=0, tries=0;
unsigned char buf[256];

typedef struct {
    enum state_t { SMSTART,SMFLAG,SMADR,SMCTRL,SMBCC1,SMDATA,SMESC,SMBCC2,SMEND} state;
    unsigned char adr;
    unsigned char ctrl;
    unsigned char bcc;
    unsigned char *data;
    unsigned char data_size;
} State;
State state;

//#include "link_layer.h"
int stuff(unsigned char *buf, int bufSize, unsigned char* dest, unsigned char *bcc){
    int size=0;
    for(unsigned int i=0; i<bufSize ; ++i){
        *bcc^=buf[i];
        if(buf[i]==FLAG){
            dest[size++]=ESCAPE;
            dest[size++]=ESCAPE_FLAG;
            break;
        }
        if(buf[i]==ESCAPE){
            dest[size++]=ESCAPE;
            dest[size++]=ESCAPE_ESCAPE;
            break;
        }
        dest[size++]=buf[i];
    }
    return size;
}
int buildFrame(unsigned char* buf, unsigned char* data,unsigned int data_size, unsigned char address, unsigned char control, unsigned char bcc2){
    
    buf[0]= FLAG;
    buf[1]= address;
    buf[2]= control;
    buf[3]= address ^ control;
    if(data==NULL){
        //is command packet.
        buf[4]= FLAG;
        return 5;
    }
    memcpy(buf+4,data,data_size);
    buf[4+data_size]=bcc2;
    buf[5+data_size]=FLAG;
    return 6+data_size;
}
int buildDataFrame(unsigned char* buf, unsigned char* data,unsigned int data_size, unsigned char address, unsigned char control){
    buf[0]= FLAG;
    buf[1]= address;
    buf[2]= control;
    buf[3]= address ^ control;
    int offset=0;
    unsigned char bcc=0;
    for(unsigned int i=0;i<data_size;++i)
        stuff(data+i,1,buf+offset,&bcc);
    buf[4+offset]=bcc;
    buf[5+offset]=FLAG;
    return 6+offset;
}


void alarmHandler(int signal){
    ++tries;
    alarmEnabled=0;
}

void state_machine(unsigned char byte,State* state){
    switch (state->state){
        case SMEND:
        case SMSTART:
            if(byte==FLAG)
                state->state=SMFLAG;
            break;
        case SMFLAG:
            if(byte==FLAG){
                break;
            }
            if(byte==ADR_TX || byte == ADR_RX){
                state->state=SMADR;
                state->adr=byte;
                break;
            }
            state->state=SMSTART;
            break;
        case SMADR:
            if(byte==FLAG){
                state->state=SMFLAG;
                break;
            }
            if(byte==CTRL_DISC || byte==CTRL_SET 
            || byte==CTRL_UA || byte == CTRL_REJ(0) 
            || byte==CTRL_RR(0) || byte == CTRL_REJ(1) 
            || byte==CTRL_RR(1) || byte == CTRL_DATA(0)
            || byte==CTRL_DATA(1)){
                state->state = SMCTRL;
                state->ctrl = byte;
                state->bcc = state->adr ^ state->ctrl;
                break;
            }
            state->state=SMSTART;
            break;
        case SMCTRL:
            if( byte == state->bcc){
                state->state=SMBCC1;
                break;
            }
            if(byte==FLAG){
                state->state = SMFLAG;
                break;
            }
            state->state=SMSTART;
            break;
        case SMBCC1:
            if(byte==FLAG){
                if(state->ctrl==CTRL_DATA(0) || state->ctrl==CTRL_DATA(1)){
                    //Received flag when expecting data.
                    state->state=SMFLAG;
                    break;
                }
                state->state=SMEND; //received valid frame.
                break;
            }
            if(state->ctrl==CTRL_DATA(0) || state->ctrl==CTRL_DATA(1)){
                state->data_size=0;
                if(byte==ESCAPE){
                    state->bcc=0;
                    state->state=SMESC;
                    break;       
                }
                state->data[state->data_size++]=byte;
                state->bcc=byte;
                state->state=SMDATA;
                break;
            }
            state->state=SMSTART;
            break;
        case SMDATA:
            if(byte==ESCAPE){
                state->state=SMESC;
                break;       
            }
            if(byte==state->bcc){
                state->state=SMBCC2;
                break;
            }
            if(byte==FLAG){
                state->state=SMFLAG;
                break;
            }
            state->state=SMSTART;
            break;
        case SMESC:
            if(byte==ESCAPE_FLAG){
                if(state->bcc==FLAG){
                    state->state=SMBCC2;
                    break;
                }
                state->bcc^=FLAG;
                state->data[state->data_size++]=FLAG;
                state->state=SMDATA;
                break;
            }
            if(byte==ESCAPE_ESCAPE){
                if(state->bcc==ESCAPE){
                    state->state=SMBCC2;
                    break;
                }
                state->bcc^=ESCAPE;
                state->data[state->data_size++]=ESCAPE;
                state->state=SMDATA;
                break;
            }
            state->state=SMSTART;
            break;
        case SMBCC2:
            if(byte==FLAG){
                state->state=SMEND;
                break;
            }
            if(byte==0){
                state->data[state->data_size++]=state->bcc;
                state->bcc=0;
                break;
            }
            state->bcc=byte; 
            state->data[state->data_size++]=state->bcc;
            state->state=SMDATA;
            break;
    }
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    connection=connectionParameters;
    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    fd = open(connection.serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(connection.serialPort);
        exit(-1);
    }
    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = connection.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 1; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 1 char received

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
    
    signal(SIGALRM,alarmHandler);

    if(connection.role==LlTx){ //Transmitter
        int receivedUA=0;
        state.state=SMSTART;
        tries=0;
        while(tries<connection.nRetransmissions && !receivedUA){
            alarm(connection.timeout);
            alarmEnabled=1;
            if(tries>0)
                printf("Timed out.\n");
            int size = buildFrame(buf,NULL,0,ADR_TX,CTRL_SET,0);
            write(fd,buf,size);
            while(alarmEnabled && !receivedUA){
                int bytes_read = read(fd,buf,PACKET_SIZE_LIMIT);
                if(bytes_read<0)
                    return -1;
                for(unsigned int i=0;i<bytes_read && !receivedUA;++i){
                    state_machine(buf[i],&state);
                    if(state.state==SMEND && state.adr==ADR_TX && state.ctrl == CTRL_UA)
                        receivedUA=1;
                }
            }
        }
        if(receivedUA)
            printf("Received UA.\n");
        return 1;
    }
    else{ // Receiver
        tries=0;
        
        state.state=SMSTART;
        int receivedSET=0;
        while(!receivedSET){
            int bytes_read = read(fd,buf,PACKET_SIZE_LIMIT);
            if(bytes_read<0)
                return -1;
            for(unsigned int i=0;i<bytes_read && !receivedSET;++i){
                state_machine(buf[i],&state);
                if(state.state==SMEND && state.adr==ADR_TX && state.ctrl == CTRL_SET)
                    receivedSET=1;
            }
        }
        int frame_size=buildFrame(buf,0,0,ADR_TX,CTRL_UA,0);
        sleep(9);
        write(fd,buf,frame_size); //sends UA reply.
        return 1;
    }
    return -1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buffer, int bufferSize)
{
    unsigned char bigBuf[bufferSize*2+6];
    int frame_size=buildDataFrame(bigBuf,buffer,bufferSize,ADR_TX,CTRL_DATA(0));
    
    for(unsigned int sent=0;sent<frame_size;){ //In case write doesnt write all bytes from the first call.
        int ret=write(fd,bigBuf+sent,frame_size-sent);
        if(ret==-1)
            return -1;
        sent+=ret;
    }
    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    int receivedPacket=0;
    state.data=packet; //State machine writes to packet buffer directly.
    while(!receivedPacket){
        int bytes_read = read(fd,packet,PACKET_SIZE_LIMIT);
        if(bytes_read<0)
            return -1;
        for(unsigned int i=0;i<bytes_read && !receivedPacket;++i){
            state_machine(packet[i],&state);
            if(state.state==SMEND && state.adr==ADR_TX && state.ctrl == CTRL_SET){
                int frame_size=buildFrame(buf,0,0,ADR_TX,CTRL_UA,0);
                write(fd,buf,frame_size); //sends UA reply.
            }
            if(state.state==SMEND && state.adr==ADR_TX && state.ctrl == CTRL_DATA(0)){ //TODO alternate frame counter bit.
                return state.data_size;
            }
            //TODO maybe handle other commands?
        }
    }
    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    return 1;
}
