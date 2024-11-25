#ifndef REQREP_H
#define REPREQ_H

enum{ 
    TYPE_REQUEST = 0,
    TYPE_REPLY = 1,
    TYPE_F = 2
}; 

typedef enum{
    TRANSPORT_SYN = 0x01,
    TRANSPORT_ACK = 0x02,
    TRANSPORT_FIN = 0x04
} TransportFlags;

#endif



