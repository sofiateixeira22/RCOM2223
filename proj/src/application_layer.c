// Application layer protocol implementation
#include <unistd.h>
#include <string.h>
#include "application_layer.h"
#include "link_layer.h"



void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    //unsigned char buf[256];
    LinkLayer ll;
    strcpy(ll.serialPort,serialPort);
    ll.baudRate = baudRate;
    ll.nRetransmissions = nTries;
    ll.timeout = timeout;
    if( 0 == strcmp(role,"tx" ) ){
        ll.role=LlTx;
        if(1!=llopen(ll)){
            sleep(1);
            llclose(0);
            return;
        }
        unsigned char buf[256];
        for(unsigned int i =0;i<256;++i){
            buf[i]=i;
        }
        printf("llwrite:%i\n",llwrite(buf,256));
    }
    else if( 0 == strcmp(role,"rx")){
        ll.role=LlRx;
        if(1!=llopen(ll)){
            sleep(1);
            llclose(0);
            return;
        }
        unsigned char buf[256];
        llread(buf);
        int flag=0;
        for(unsigned int i =0;i<256;++i){
            if(buf[i]!=i){
                printf("llread[%i] error.\n",i);
                flag=1;
            }
        }
        if(!flag)
            printf("llread succeeded.\n");
    }
    sleep(1);
    llclose(0);
}
