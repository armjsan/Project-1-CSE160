#include "../../includes/packet.h"
#include "../../includes/socket.h"

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
    socket_store_t sockets[MAX_NUM_OF_SOCKETS];

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

    // send pack based on state ^
    void sendPacket(socket_store_t* socket) {
        pack packet;
        uint8_t type;

        switch(socket->state) {
            case SYN_SENT:
                type = TRANSPORT_SYN;
                break;
            case SYN_RCVD:
                type = TRANSPORT_SYN_ACK;
                break;
            case ESTABLISHED:
                type = TRANSPORT_DATA;
                break;
            case FIN_WAIT:
                type = TRANSPORT_FIN;
                break;
            default:
                return;
        }
        
        // pack fields
        packet.protocol = PROTOCOL_TRANSPORT; 
        packet.dest = socket->dest.addr;
        packet.src = TOS_NODE_ID;
        packet.seq = socket->lastSent; 
        
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

    event void TimeoutTimer.fired() {
        // gonna handle the  retransmissions here
    }
}