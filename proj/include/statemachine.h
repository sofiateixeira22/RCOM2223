// Statemachine header.

#ifndef _STATEMACHINE_H_
#define _STATEMACHINE_H_

typedef struct {
    enum state_t { SMSTART,SMFLAG,SMADR,SMCTRL,SMBCC1,SMDATA,SMESC,SMBCC2,SMEND,SMREJ} state;
    unsigned char adr;
    unsigned char ctrl;
    unsigned char bcc;
    unsigned char *data;
    unsigned int data_size;
} State;
State state;

void state_machine(unsigned char byte, State* state);

#endif // _STATEMACHINE_H_