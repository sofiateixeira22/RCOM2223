// Macros header.

#ifndef _MACROS_H_
#define _MACROS_H_

//Application Layer
#define PACKET_SIZE (1024)
#define CONTROL_START (2)
#define CONTROL_END (3)
#define CONTROL_DATA (1)
#define TYPE_FILESIZE (0)

//Link Layer
#define FLAG (0b01111110)
#define ESCAPE (0x7d)
#define ESCAPE_FLAG (0x5e)
#define ESCAPE_ESCAPE (0x5d)
#define ADR_TX (0b00000011)
#define ADR_RX (0b00000001)
#define CTRL_SET (0b00000011)
#define CTRL_DISC (0b00001011)
#define CTRL_UA (0b00000111)
#define CTRL_RR(R) ((R)%2?0b10000101:0b00000101)
#define CTRL_REJ(R) ((R)%2?0b10000001:0b00000001)
#define CTRL_DATA(S) ((S)%2?0b01000000:0b00000000)
#define PACKET_SIZE_LIMIT (128)

#endif // _MACROS_H_
