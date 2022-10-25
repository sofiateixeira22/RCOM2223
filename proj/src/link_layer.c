// Link layer protocol implementation

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include "link_layer.h"
#include "macros.h"
#include "statemachine.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

LinkLayer connection;
struct termios oldtio;
struct termios newtio;
int fd;
int receivedDISC = 0;
unsigned char alarmEnabled=0, tries=0;
unsigned char buf[128], *bigBuf =NULL;
unsigned long bigBufsize=0;
unsigned char data_s_flag = 0;

int stuff(const unsigned char *buffer, int bufSize, unsigned char* dest, unsigned char *bcc){
    int size=0;
    for(unsigned int i=0; i<bufSize ; ++i){
        if(bcc!=NULL)
            *bcc^=buffer[i];
        if(buffer[i]==FLAG){
            dest[size++]=ESCAPE;
            dest[size++]=ESCAPE_FLAG;
            break;
        }
        if(buffer[i]==ESCAPE){
            dest[size++]=ESCAPE;
            dest[size++]=ESCAPE_ESCAPE;
            break;
        }
        dest[size++]=buffer[i];
    }
    return size;
}
int buildCommandFrame(unsigned char* buffer, unsigned char address, unsigned char control){
    buffer[0]= FLAG;
    buffer[1]= address;
    buffer[2]= control;
    buffer[3]= address ^ control;
    //is command packet.
    buffer[4]= FLAG;
    return 5;
}
int buildDataFrame(unsigned char* framebuf, const unsigned char* data,unsigned int data_size, unsigned char address, unsigned char control){
    framebuf[0]= FLAG;
    framebuf[1]= address;
    framebuf[2]= control;
    framebuf[3]= address ^ control;
    int offset=0;
    unsigned char bcc=0;
    for(unsigned int i=0;i<data_size;++i){
        offset+=stuff(data+i,1,framebuf+offset+4,&bcc);
    }
    offset+=stuff(&bcc,1,framebuf+offset+4,NULL);
    framebuf[4+offset]=FLAG;
    return 5+offset;
}


void alarmHandler(int signal){
    ++tries;
    alarmEnabled=0;
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
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
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
            int size = buildCommandFrame(buf,ADR_TX,CTRL_SET);
            printf("-> Sent SET.\n");
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
            printf("-> Received UA.\n");
        else
            return -1;
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
                if(state.state==SMEND && state.adr==ADR_TX && state.ctrl==CTRL_DISC) {
			receivedDISC = 1;
			printf("-> Received DISC.\n");
			return -1;
		}
            }
        }
        if(receivedSET) printf("-> Received Set.\n");
        int frame_size=buildCommandFrame(buf,ADR_TX,CTRL_UA);
        //sleep(9);
        write(fd,buf,frame_size); //sends UA reply.
        printf("-> Sent UA.\n");
        return 1;
    }
    return -1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buffer, int bufferSize)
{
    if(bigBufsize < bufferSize*2+10){
        if(bigBufsize==0)
            bigBuf=malloc(bufferSize*2+10);
        else
            bigBuf=realloc(bigBuf,bufferSize*2+10);
    }
    int frame_size=buildDataFrame(bigBuf,buffer,bufferSize,ADR_TX,CTRL_DATA(data_s_flag));
    for(unsigned int sent=0;sent<frame_size;){ //In case write doesnt write all bytes from the first call.
        int ret=write(fd,bigBuf+sent,frame_size-sent);
        if(ret==-1)
            return -1;
        sent+=ret;
    }
    int receivedPacket=0, resend=0, retransmissions=0;
    state.data=NULL; //State machine writes to packet buffer directly.
    
    alarmEnabled=1;
    alarm(connection.timeout);
    while(!receivedPacket){
        if(!alarmEnabled){
            resend=1;
            alarmEnabled=1;
            alarm(connection.timeout);
        }
        if(resend){
            if(retransmissions>0) printf("Timed out.\n");
            if(retransmissions == connection.nRetransmissions){
                printf("Exceeded retransmission limit.\n");
                return -1;
            }
            
            for(unsigned int sent=0;sent<frame_size;){ //In case write doesnt write all bytes from the first call.
                int ret=write(fd,bigBuf+sent,frame_size-sent);
                if(ret==-1)
                    return -1;
                sent+=ret;
            }
            resend = 0;
            retransmissions++;
        }
        int bytes_read = read(fd,buf,PACKET_SIZE_LIMIT);
        if(bytes_read<0)
            return -1;
        for(unsigned int i=0;i<bytes_read && !receivedPacket && alarmEnabled;++i){ //TODO avoid discarding reads after valid packet.
            state_machine(buf[i],&state);
            if(state.state==SMEND){
                if(state.adr==ADR_TX && (state.ctrl == CTRL_RR(0) || state.ctrl == CTRL_RR(1))){ //Receiver Ready for next.
                    receivedPacket = 1;
                    if(state.ctrl == CTRL_RR(data_s_flag))//Requesting next packet.
                        printf("Requesting next packet.\n");
                    resend = 0;
                }
                if(state.adr==ADR_TX && state.ctrl == CTRL_REJ(data_s_flag) ){//Requesting retransmission.
                    printf("Requesting retransmission.\n");
		    retransmissions=0;
                }
            }
            //TODO maybe handle other commands?
        }
    }
    data_s_flag= data_s_flag?0:1;
    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    if(bigBufsize < PACKET_SIZE_LIMIT){
        if(bigBufsize==0)
            bigBuf=malloc(PACKET_SIZE_LIMIT);
        else
            bigBuf=realloc(bigBuf,PACKET_SIZE_LIMIT);
    }
    int receivedPacket=0;
    state.data=packet; //State machine writes to packet buffer directly.
    while(!receivedPacket){
        int bytes_read = read(fd,bigBuf,PACKET_SIZE_LIMIT);
        if(bytes_read<0)
            return -1;
        for(unsigned int i=0;i<bytes_read && !receivedPacket;++i){
            state_machine(bigBuf[i],&state);
            if(state.state==SMREJ && state.adr==ADR_TX){
                int frame_size=buildCommandFrame(buf,ADR_TX,(state.ctrl==CTRL_DATA(0)?CTRL_REJ(0):CTRL_REJ(1)));
                write(fd,buf,frame_size); //sends REJ reply.
                printf("-> Sent REJ.\n");
            }
            if(state.state==SMEND && state.adr==ADR_TX && state.ctrl == CTRL_SET){
                int frame_size=buildCommandFrame(buf,ADR_TX,CTRL_UA);
                write(fd,buf,frame_size); //sends UA reply.
                printf("-> Sent UA.\n");
            }
            if(state.state==SMEND && state.adr==ADR_TX){
                if(state.ctrl == CTRL_DATA(data_s_flag)){
                    data_s_flag=data_s_flag?0:1;
                    int frame_size=buildCommandFrame(buf,ADR_TX,CTRL_RR(data_s_flag));
                    write(fd,buf,frame_size);
                    printf("-> Sent RR %i.\n",data_s_flag);
                    return state.data_size;
                }
                else{
                    int frame_size=buildCommandFrame(buf,ADR_TX,CTRL_RR(data_s_flag));
                    write(fd,buf,frame_size);
                    printf("-> Sent RR %i requesting retransmission.\n",data_s_flag);
                }
            }
            if(state.ctrl==CTRL_DISC) {
                receivedDISC = 1;
                int frame_size=buildCommandFrame(buf,ADR_TX,(state.ctrl==CTRL_DATA(0)?CTRL_REJ(0):CTRL_REJ(1)));
                write(fd,buf,frame_size); //sends REJ reply.
                printf("-> Received DISC.\n");
                return -1;
                break;
            }
        }
    }
    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    signal(SIGALRM,alarmHandler);
    if(bigBufsize>0)
        free(bigBuf);
    
    if(connection.role==LlTx) { //Transmitter

        int receivedDISC_tx=0;
        tries=0;
        while(tries<connection.nRetransmissions && !receivedDISC_tx){
            alarm(connection.timeout);
            alarmEnabled=1;
            if(tries>0)
                printf("Timed out.\n");
            int size = buildCommandFrame(buf,ADR_TX,CTRL_DISC);
            printf("-> Sent DISC.\n");
            write(fd,buf,size);
            while(alarmEnabled && !receivedDISC_tx){
                int bytes_read = read(fd,buf,PACKET_SIZE_LIMIT);
                if(bytes_read<0)
                    return -1;
                for(unsigned int i=0;i<bytes_read && !receivedDISC_tx;++i){
                    state_machine(buf[i],&state);
                    if(state.state==SMEND && state.adr==ADR_TX && state.ctrl == CTRL_DISC)
                        receivedDISC_tx=1;
                    if(state.state==SMEND && state.adr==ADR_TX && (state.ctrl == CTRL_RR(0) || state.ctrl == CTRL_RR(1) || state.ctrl == CTRL_REJ(0) || state.ctrl == CTRL_REJ(1))){
                        tries=0; //reset tries as receiver was still in llread.
                    }
                }
            }
        }
        if(receivedDISC_tx){ 
            printf("-> Received DISC.\n");
            int frame_size=buildCommandFrame(buf,ADR_TX,CTRL_UA);
            write(fd,buf,frame_size); //sends UA reply.
            printf("-> Sent UA.\n");
            sleep(1);
        }

    } else { //Receiver

        while(!receivedDISC){
            int bytes_read = read(fd,buf,PACKET_SIZE_LIMIT);
            if(bytes_read<0)
                return -1;
            for(unsigned int i=0;i<bytes_read && !receivedDISC;++i){
                state_machine(buf[i],&state);
                if(state.state==SMEND && state.adr==ADR_TX && state.ctrl == CTRL_DISC)
                    receivedDISC=1;
            }
        }
        if(receivedDISC) printf("-> Received DISC.\n");
        int frame_size=buildCommandFrame(buf,ADR_TX,CTRL_DISC);
        write(fd,buf,frame_size); //sends DISC reply.
        printf("-> Sent DISC.\n");

        int receivedUA=0;
        while(!receivedUA){
            int bytes_read = read(fd,buf,PACKET_SIZE_LIMIT);
            if(bytes_read<0)
                return -1;
            for(unsigned int i=0;i<bytes_read && !receivedUA;++i){
                state_machine(buf[i],&state);
                if(state.state==SMEND && state.adr==ADR_TX && state.ctrl == CTRL_UA)
                    receivedUA=1;
            }
        }
        if(receivedUA) printf("-> Received UA.\n");
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    return 1;
}
