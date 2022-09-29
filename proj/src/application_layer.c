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
        
    }
    else if( 0 == strcmp(role,"rx")){
        ll.role=LlRx;
        if(1!=llopen(ll)){
            sleep(1);
            llclose(0);
            return;
        }
        
    }
}
