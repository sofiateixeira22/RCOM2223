// Application layer protocol implementation
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include "application_layer.h"
#include "link_layer.h"

#define FLAG 0b01111110
unsigned char alarmEnabled=0, tries=0;

//#include "link_layer.h"
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


void alarmHandler(int signal){
    ++tries;

}


void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    unsigned char buf[256];
    LinkLayer ll;
    strcpy(ll.serialPort,serialPort);
    ll.baudRate = baudRate;
    ll.nRetransmissions = nTries;
    ll.timeout = timeout;
    
    signal(SIGALRM,alarmHandler);



    if( 0 == strcmp(role,"tx" ) ){
        if(1!=llopen(ll)){
            return;
        }
        
        ll.role=LlTx;
        tries=0;
        //llopen()
        while(tries<nTries){
            
            //llwrite();
        }

    }
    else if( 0 == strcmp(role,"rx")){
        ll.role=LlRx;
        
    }
}
