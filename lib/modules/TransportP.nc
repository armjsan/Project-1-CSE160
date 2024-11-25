#include "../../includes/packet.h"
#include "../../includes/socket.h"
#include "../../includes/ReQRep.h"

module TransportP {
    provides interface Transport;
    
    uses {
        interface Timer<TMilli> as TimeoutTimer;
        interface IPmod;
        interface Random;
    }
}

implementation {
    // socket store and management
    socket_store_t sockets[MAX_NUM_OF_SOCKETS]; //!!!

   //  stop and wait stuff:
    // typedef nx_struct transport_packet {
    //     nx_uint8_t type;  // need this for ACKs
    //     nx_uint16_t seq;  
    //     nx_uint8_t payload[TRANSPORT_MAX_PAYLOAD_SIZE];
    // } transport_packet_t;
    
    enum {
        TIMEOUT = 20000,  // 20 seconds timeout
        MAX_RETRIES = 5,
        WINDOW_SIZE = 10  
    };

    // enum { 
    //     WAITING_FOR_ACK = 1,
    //     READY_TO_SEND = 2
    // }

    // event message_t* Receiver.receiver(message_t* msg, void* payload) { 
    //     return msg; 
    // }

    // helper functions
    // find free socket
    socket_t findSocket() {
        uint8_t i;
        for(i = 0; i < MAX_NUM_OF_SOCKETS; i++) {
            if(sockets[i].state == CLOSED) {
                return i;
            }
        }
        return SOCKET_ERROR; // no socket is found 
    }

    socket_store_t* findSocketByAddr(socket_addr_t* src, socket_addr_t* dest){
        socket_t fd;
        for (fd = 0; fd < MAX_NUM_OF_SOCKETS; fd++){
            socket_store_t* socket = &socketStore[fd]; //

            if (socket->state == CLOSED){
                continue;
            }
            if (socket->src.port == dest->port && socket->src.addr == dest->addr
            && socket->dest.port == src->port && socket->dest.addr == src-> addr){
                return socket;
            }
        }

        return NULL;
    }

    // send pack based on state ^
    void sendPacket(socket_store_t* socket) {
        pack packet;
        // uint8_t type;

        //might use memcpy! Who knows
        transport_payload* payload = (transport_payload*)packet.payload; //setting payload directly to packet payload
        payload->src = (socket_addr_t){.port = socket->src, .addr = TOS_NODE_ID}; //source of where we are sending it from
        payload->dest = socket->dest; //where we want to g
        payload->seq = socket->lastSent++;


        switch(socket->state) {
            case SYN_SENT:
                payload->flag = TRANSPORT_SYN;
                break;
            case SYN_RCVD:
                payload->flag = TRANSPORT_SYN | TRANSPORT_ACK;
                break;
            case ESTABLISHED:
                payload->flag = TRANSPORT_ACK;
                break;
            case FIN_WAIT:
                payload->flag = TRANSPORT_FIN;
                break;
            default:
                return;
        }
        
        // payload.flag = type;
        
        // pack fields
        packet.protocol = PROTOCOL_TRANSPORT; 
        packet.dest = socket->dest.addr;
        packet.src = TOS_NODE_ID;
        packet.seq = socket->lastSent;
        // memcpy(packet.payload, &payload, sizeof(payload))
        // packet.type = type;
        
        // send with IP layer
        call IPmod.send(&packet, socket->dest.addr);
        
        //start timeout timer
        if(type != TRANSPORT_DATA) {
            call TimeoutTimer.startOneShot(TIMEOUT);
        }
    }

    // interface implementation
    // set all sockets to closed 
    command error_t Transport.start() {
        uint8_t i;
        for(i = 0; i < MAX_NUM_OF_SOCKETS; i++) {
            sockets[i].state = CLOSED;
        }
        return SUCCESS;
    }

    // setup 
    command socket_t Transport.socket() {
        socket_t fd = findSocket();
        if(fd != SOCKET_ERROR) {
            sockets[fd].state = CLOSED;
            sockets[fd].lastSent = 0;
            sockets[fd].lastRcvd = 0;
        }
        return fd;
    }

    command error_t Transport.bind(socket_t fd, socket_addr_t *addr) {
        if(fd >= MAX_NUM_OF_SOCKETS || sockets[fd].state != CLOSED) {
            return FAIL;
        }
        sockets[fd].src = *addr; 
        sockets[fd].state = LISTEN;
        return SUCCESS;
    }

    // trying to initiate conenction to a rmemote address 
    command error_t Transport.connect(socket_t fd, socket_addr_t *addr) {
        if(fd >= MAX_NUM_OF_SOCKETS || sockets[fd].state != CLOSED) {
            return FAIL;
        }
        sockets[fd].dest = *addr;
        sockets[fd].state = SYN_SENT;
        sendPacket(&sockets[fd]);
        return SUCCESS;
    }

    // tear down
    // close connection
    command error_t Transport.close(socket_t fd) {
        if(fd >= MAX_NUM_OF_SOCKETS || sockets[fd].state == CLOSED) {
            return FAIL;
        }
        // need to add clean up
        sockets[fd].state = FIN_WAIT; 
        sendPacket(&sockets[fd]);
        return SUCCESS;
    }

    command error_t Transport.receive(pack* package){
        if (packet->protocl != PROTOCOL_TRANSPORT){
            return FAIL
        }

        transport_packet* payload = (transport_packet*)packet->payload;

        socket_store_t* socket = findSocketByAddr(&payload->src, &payload->desr);
        if (socket == NULL){
            return FAIL;
            // dbg(TRANSPORT_CHANNEL, "")
        }

        switch (socket->state){
            case Listen:
                if(payload->flag & TRANSPORT_SYN){
                    sendPacket(socket);
                    socket->state = SYN_RCVD;
                }
                break;
            case SYN_SENT:
                if ((payload->flag & TRANSPORT_SYN) && (payload->flag & TRANSPORT_ACK)){
                    sendPacket(socket);
                    socket->state = ESTABLISHED;
                }
                break;
            case SYN_RCVD:
                if(payload->flag & TRANSPORT_ACK){
                    socket->state = ESTABLISHED;
                }
                break;
            case ESTABLISHED:
                sdc
            case FIN_WAIT:
                if(payload->flag & TRANSPORT_ACK){
                    socket->state = CLOSED;
                }
        }
    }

    event void TimeoutTimer.fired() {
        // gonna handle the  retransmissions here
    }
}