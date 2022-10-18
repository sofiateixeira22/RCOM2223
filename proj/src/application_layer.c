// Application layer protocol implementation
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include "application_layer.h"
#include "link_layer.h"

#define PACKET_SIZE (1024)
#define CONTROL_START (2)
#define CONTROL_END (3)
#define CONTROL_DATA (1)
#define TYPE_FILESIZE (0)

unsigned char appbuf[PACKET_SIZE+30];

int read_next_TLV(unsigned char *addr, unsigned char* t, unsigned char* l, unsigned char** v){
    *t=addr[0];
    *l=addr[1];
    *v=addr+2;
    return 2 + *l; //returns size of the TLV.
}


void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename)
{
    LinkLayer ll;
    strcpy(ll.serialPort,serialPort);
    
    if(strcmp(role,"tx")==0) {
        ll.role = LlTx;
    } else if(strcmp(role, "rx")==0) {
        ll.role = LlRx;
    } else {
        printf("Application Layer: Not a defined role.\n");
        return;
    }

    ll.baudRate = baudRate;
    ll.nRetransmissions = nTries;
    ll.timeout = timeout;

    if(llopen(ll)<0) {
        printf("ERROR Application Layer: An error occurred while opening the connection.\n");
        llclose(0);
        return;
    }

    if(ll.role == LlTx) {
        FILE* file = fopen(filename, "r");
        
        if(!file) {
            printf("ERROR Application Layer: Could not open file to be sent.\n");
            return;
        } else {
            printf("Application Layer: Successfully opened file to be sent.\n");
        }

        fseek(file, 0L, SEEK_END);  
        long int file_size = ftell(file);
        fseek(file,0,SEEK_SET);
        printf("Application Layer File Size: %li bytes\n", file_size);
        appbuf[0]=CONTROL_START;
        appbuf[1]=TYPE_FILESIZE;
        appbuf[2]=sizeof(long);
        /*for(unsigned int i=0;i<8;++i){
            appbuf[3+i]= (unsigned char)((((unsigned long)file_size) % (2^(8*(i+1)))) >> 8*i);
        }*/
        *((long*)(appbuf+3))=file_size;
        llwrite(appbuf,10);
        unsigned char failure=0;
        unsigned long bytes_sent =0;
        for(unsigned char i=0;bytes_sent<file_size;++i){
            unsigned long file_bytes = fread(appbuf+4, 1, (file_size-bytes_sent<PACKET_SIZE-1000? file_size-bytes_sent : PACKET_SIZE-1000), file);
            
            if(file_bytes!=(file_size-bytes_sent<PACKET_SIZE-1000? file_size-bytes_sent:PACKET_SIZE-1000)){
                printf("File read failure. file_bytes:%lu\n",file_bytes);
                failure=1;
                break;
            }
            appbuf[0]=CONTROL_DATA;
            appbuf[1]=i;
            appbuf[2]=file_bytes>>8;
            appbuf[3]=file_bytes%256;
            if(-1==llwrite(appbuf,file_bytes+4)){
                printf("llwrite failure.\n");
                failure=1;
                break;
            }
            printf("applicationLayer: sent packet %i.\n",i);
        }
        if(!failure){
            appbuf[0]=CONTROL_END;
            if(-1==llwrite(appbuf,1)){
                printf("llwrite on end packet failure.\n");
            }else{
                printf("Finished sending.\n");
            }
        }
        fclose(file);
    } else if(ll.role == LlRx) {
        unsigned long filesize=0,size_received=0;
        int bytes_read = llread(appbuf);
        unsigned char t,l,*v;
        if(appbuf[0] == CONTROL_START){
            int offset=1;
            for(;offset<bytes_read;){
                offset+=read_next_TLV(appbuf+offset,&t,&l,&v);
                if(t==TYPE_FILESIZE){
                    filesize=*((unsigned long*)v);
                    printf("Filesize:%li",filesize);
                }
            }
            FILE* file = fopen(filename, "w");
            if(!file) {
                printf("\nERROR Application Layer: Could not open file to write in.\n");
                return;
            } else {
                printf("\nApplication Layer: Received control packet and created file to write.\n");
            }
            unsigned char early_end =0, last_sequence_number=0;
            for(;size_received<filesize;){
                int numbytes=llread(appbuf);
                if(numbytes<1){
                    if(numbytes==-1){
                        printf("llread error.\n");
                        break;
                    }else
                        printf("Received a packet that is too small. (%i bytes)\n",numbytes);
                }
                if(appbuf[0]==CONTROL_END){
                    printf("Disconnected before end of file.\n");
                    early_end=1;
                    break;
                }
                if(appbuf[0]==CONTROL_DATA){
                    if(numbytes<5)
                        printf("Received a data packet that is too small. (%i bytes)\n",numbytes);
                    if(appbuf[1]!=last_sequence_number){
                        printf("Received packet with wrong sequence number. Was %i, expected:%i\n",appbuf[1],last_sequence_number-1);
                    }else{
                        unsigned long size=appbuf[3]+appbuf[2]*256;
                        if(size!=numbytes-4)
                            printf("Packet length didn't match header. Was %i, expected %lu.\n",numbytes-4,size);
                        fwrite(appbuf+4,1,size,file);
                        size_received+=size;
                        printf("Received packet numbered %i.\n",last_sequence_number++);
                    }
                }
            }
            if(!early_end){
                int numbytes=llread(appbuf);
                if(numbytes<1){
                    if(numbytes==-1)
                        printf("llread error.\n");
                    else
                        printf("Received a packet that is too small. (%i bytes)\n",numbytes);
                }
                if(appbuf[0]!=CONTROL_END){
                    printf("Received an unexpected packet. Expected end control packet.\n");
                }else{
                    printf("Received end packet. Disconnecting\n");
                }
            }
            fclose(file);
        }else{
            printf("Transmission didn't start with a start packet.\n");
            for(unsigned int i=0;i<10;++i)
                printf("%i ",appbuf[i]);
        }
    }
    llclose(0);
    sleep(1);
}
