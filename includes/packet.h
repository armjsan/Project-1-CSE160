//Author: UCM ANDES Lab
//$Author: abeltran2 $
//$LastChangedBy: abeltran2 $

#ifndef PACKET_H
#define PACKET_H


#include "protocol.h"
#include "channels.h"
#include "ReQRep.h"
#include "socket.h"

#define MAX_TUPLE 5

enum{
	// had to adjust packet header size becasue I was getting a segmentation fault
	PACKET_HEADER_LENGTH = 10,
	PACKET_MAX_PAYLOAD_SIZE = 28 - PACKET_HEADER_LENGTH,
	MAX_TTL = 15
};


typedef nx_struct pack{
	nx_uint16_t dest;
	nx_uint16_t src;
	nx_uint16_t seq;	//Sequence Number
	nx_uint8_t TTL;		//Time to Live
	nx_uint8_t protocol;
	nx_uint8_t type; // Will be used as type (reply or request) for neighbor and Flood Src for flooding
	nx_uint8_t fdest; // Flood Destination
	nx_uint8_t payload[PACKET_MAX_PAYLOAD_SIZE];
}pack;

/*
 * logPack
 * 	Sends packet information to the general channel.
 * @param:
 * 		pack *input = pack to be printed.
 */
void logPack(pack *input){
	dbg(GENERAL_CHANNEL, "Src: %hhu Dest: %hhu Seq: %hhu TTL: %hhu Protocol:%hhu Type:%hhu Fdest:%hhu Payload: %s\n",
	input->src, input->dest, input->seq, input->TTL, input->protocol, input->type,input->fdest, input->payload);
}

typedef nx_struct {
	nx_uint16_t neighbor;
	nx_uint8_t cost;
}tuple_t;

typedef nx_struct LSA{
	nx_uint16_t src;
	nx_uint16_t seq;
	tuple_t tupleList[MAX_TUPLE];
	nx_uint8_t payload[];
} LSA;

 typedef nx_struct transport_packet {
        socket_addr_t src;
		socket_addr_t dest;
		uint8_t flag;  // need this for ACKs
        uint16_t seq;  
		uint16_t windowSIZE;
        nx_uint8_t payload[TRANSPORT_MAX_PAYLOAD_SIZE];
    } transport_packet_t;

enum{
	AM_PACK=6,
	AM_LSA=7
};

#endif
