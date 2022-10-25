// Application layer protocol implementation
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include "application_layer.h"
#include "link_layer.h"
#include "macros.h"

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
        printf("ERROR: Not a defined role.\n");
        return;
    }

    ll.baudRate = baudRate;
    ll.nRetransmissions = nTries;
    ll.timeout = timeout;

    printf("\nExecuting llopen:\n\n");

    if(llopen(ll)<0) {
        printf("ERROR: An error occurred while opening the connection.\n");
        llclose(0);
        return;
    }

    if(ll.role == LlTx) {
        FILE* file = fopen(filename, "r");
        
        if(!file) {
            printf("ERROR: Could not open file to be sent.\n");
            return;
        } else {
            printf("-> Successfully opened file to be sent.\n");
        }

        fseek(file, 0L, SEEK_END);  
        long int file_size = ftell(file);
        fseek(file,0,SEEK_SET);
        printf("File Size: %li bytes\n", file_size);
        appbuf[0]=CONTROL_START;
        appbuf[1]=TYPE_FILESIZE;
        appbuf[2]=sizeof(long);

        printf("\nExecuting llwrite:\n\n");

        *((long*)(appbuf+3))=file_size;
        llwrite(appbuf,10);
        unsigned char failure=0;
        unsigned long bytes_sent =0;
        for(unsigned char i=0;bytes_sent<file_size;++i){
            unsigned long file_bytes = fread(appbuf+4, 1, (file_size-bytes_sent<PACKET_SIZE? file_size-bytes_sent : PACKET_SIZE), file);
            
            if(file_bytes!=(file_size-bytes_sent<PACKET_SIZE? file_size-bytes_sent:PACKET_SIZE)){
                printf("ERROR: File read failure. file_bytes:%lu, file_size:%lu, bytes_sent:%lu\n",file_bytes,file_size,bytes_sent);
                failure=1;
                break;
            }
            appbuf[0]=CONTROL_DATA;
            appbuf[1]=i;
            appbuf[2]=file_bytes>>8;
            appbuf[3]=file_bytes%256;
            if(-1==llwrite(appbuf,file_bytes+4)){
                printf("ERROR: llwrite failure.\n");
                failure=1;
                break;
            }
            printf("Sent packet %i.\n",i);
            bytes_sent+=file_bytes;
        }
        if(!failure){
            appbuf[0]=CONTROL_END;
            if(-1==llwrite(appbuf,1)){
                printf("ERROR: llwrite on end packet failure.\n");
            }else{
                printf("-> Finished sending.\n");
            }
        }
        fclose(file);

    } else if(ll.role == LlRx) {
        unsigned long filesize=0,size_received=0;
        printf("\nExecuting llread:\n\n");
        int bytes_read = llread(appbuf);
        unsigned char t,l,*v;
        if(appbuf[0] == CONTROL_START){
            int offset=1;
            for(;offset<bytes_read;){
                offset+=read_next_TLV(appbuf+offset,&t,&l,&v);
                if(t==TYPE_FILESIZE){
                    filesize=*((unsigned long*)v);
                    printf("File Size:%li",filesize);
                }
            }
            FILE* file = fopen(filename, "w");
            if(!file) {
                printf("ERROR: Could not open file to write in.\n");
                return;
            } else {
                printf("-> Received control packet and created file to write.\n");
            }
            unsigned char early_end =0, last_sequence_number=0;
            for(;size_received<filesize;){
                int numbytes=llread(appbuf);
                if(numbytes<1){
                    if(numbytes==-1){
                        printf("ERROR: llread failure.\n");
                        break;
                    }else
                        printf("ERROR: Received a packet that is too small. (%i bytes)\n",numbytes);
                }
                if(appbuf[0]==CONTROL_END){
                    printf("ERROR: Disconnected before end of file.\n");
                    early_end=1;
                    break;
                }
                if(appbuf[0]==CONTROL_DATA){
                    if(numbytes<5)
                        printf("ERROR: Received a data packet that is too small. (%i bytes)\n",numbytes);
                    if(appbuf[1]!=last_sequence_number){
                        printf("ERROR: Received packet with wrong sequence number. Was %i, expected:%i\n",appbuf[1],last_sequence_number-1);
                    }else{
                        unsigned long size=appbuf[3]+appbuf[2]*256;
                        if(size!=numbytes-4)
                            printf("ERROR: Packet length didn't match header. Was %i, expected %lu.\n",numbytes-4,size);
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
                        printf("ERROR: llread failure.\n");
                    else
                        printf("ERROR: Received a packet that is too small. (%i bytes)\n",numbytes);
                }
                if(appbuf[0]!=CONTROL_END){
                    printf("ERROR: Received an unexpected packet. Expected end control packet.\n");
                }else{
                    printf("-> Received end packet. Disconnecting\n");
                }
            }
            fclose(file);
        }else{
            printf("ERROR: Transmission didn't start with a start packet.\n");
            for(unsigned int i=0;i<10;++i)
                printf("%i ",appbuf[i]);
        }
    }

    printf("\nExecuting llclose:\n\n");
    llclose(0);
    sleep(1);
}
