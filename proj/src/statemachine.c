#include <stdio.h>
#include <string.h>
#include "macros.h"
#include "statemachine.h"

void state_machine(unsigned char byte,State* state){
    switch (state->state){
        case SMREJ:
        case SMEND:
            state->state=SMSTART;
        case SMSTART:
            if(byte==FLAG)
                state->state=SMFLAG;
            break;
        case SMFLAG:
            state->data_size=0;
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
            if((state->ctrl==CTRL_DATA(0) || state->ctrl==CTRL_DATA(1) ) && state->data != NULL){
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
            if(byte==FLAG){
                state->state=SMREJ;
                break;
            }
            if(byte==state->bcc){
                state->state=SMBCC2;
                break;
            }
            state->data[state->data_size++]=byte;
            state->bcc^=byte;
            break;
        case SMESC:
            if(byte==FLAG){
                state->state=SMREJ;
                break;
            }
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
            if(byte==ESCAPE){
                state->data[state->data_size++]=state->bcc;    
                state->bcc=0;
                state->state=SMESC;
                break;
            }
            state->data[state->data_size++]=state->bcc;
            state->data[state->data_size++]=byte;
            state->bcc=byte;
            state->state=SMDATA;
            break;
    }
}