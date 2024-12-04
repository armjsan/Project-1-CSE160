#include "../../includes/packet.h"
#include "../../includes/socket.h"
#include "../../includes/ReQRep.h"

module TransportP {
    provides interface Transport;
    
    uses {
        interface Timer<TMilli> as TimeoutTimer;
        interface Timer<TMilli> as RTTTimer;
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
        uint8_t i;//this is an FD
        for(i = 0; i < MAX_NUM_OF_SOCKETS; i++) {
            if(sockets[i].state == CLOSED) {
                return i;
            }
        }
        return SOCKET_ERROR; // no socket is found 
    }
    //find the file descriptor
    socket_t findSocketByAddr(socket_addr_t* src, socket_addr_t* dest){
        socket_t fd;
        for (fd = 0; fd < MAX_NUM_OF_SOCKETS; fd++){
            socket_store_t* socket = &sockets[fd]; //

            if (socket->state == CLOSED){
                continue;
            }
            if (socket->src.port == dest->port && socket->src.addr == dest->addr
            && socket->dest.port == src->port && socket->dest.addr == src-> addr){
                return fd;
            }
        }

        return SOCKET_ERROR;
    }
    //gets that particular socket
    socket_store_t* getSocket(socket_t fd){
        if (fd >= MAX_NUM_OF_SOCKETS){
            return NULL;
        }
        return &sockets[fd];
    }
    void updateAdvertWindow(socket_store_t* socket){
        socket->effectiveWindow = SOCKET_BUFFER_SIZE - (socket->lastRcvd - socket->lastRead) % SOCKET_BUFFER_SIZE;
    }


    // send pack based on state ^
    void sendPacket(socket_store_t* socket) {
        while ((socket->lastSent - socket->lastAck) < socket->cwnd){
        pack packet;
        transport_packet* payload;
        uint16_t availableData;
        uint16_t bytesSend;
        uint16_t i;
        uint8_t index;
        

        
        payload = (transport_packet*)packet.payload; //setting payload directly to packet payload
        payload->src = (socket_addr_t){.port = socket->src, .addr = TOS_NODE_ID}; //source of where we are sending it from
        payload->dest = socket->dest; //where we want to g
        payload->seq = socket->lastSent++;

        // payload->windowSIZE = SOCKET_BUFFER_SIZE - (socket->lastRcvd - socket->lastRead) % SOCKET_BUFFER_SIZE;
        payload->windowSIZE = socket->effectiveWindow;


        switch(socket->state) {
            case SYN_SENT:
                payload->flag = TRANSPORT_SYN;
                socket->startTime;
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
            case CLOSE_WAIT:
            payload->flag = TRANSPORT_ACK;
                break;
            case LAST_ACK:
                payload->flag = TRANSPORT_FIN;
                break;
            default:
                return;
        }

        availableData = socket->lastWritten - socket->lastSent;
        if (availableData > TRANSPORT_MAX_PAYLOAD_SIZE){
            bytesSend = TRANSPORT_MAX_PAYLOAD_SIZE;
        }
        else{
            bytesSend = availableData;
        }

        for (i = 0; i < bytesSend; i++){
            index = (socket->lastSent + i) % SOCKET_BUFFER_SIZE;
            payload->data[i] = socket->sendBuff[index];
        }
        
        
        // pack fields
        packet.protocol = PROTOCOL_TRANSPORT; 
        packet.dest = socket->dest.addr;
        packet.src = TOS_NODE_ID;
        packet.seq = socket->lastSent;
        
        // send with IP layer
        call IPmod.send(&packet, socket->dest.addr);

        socket->sendTimes[socket->lastSent] = call RTTTimer.getNow();
        
        //start timeout timer
        if(type != TRANSPORT_DATA) {
            call TimeoutTimer.startOneShot(TIMEOUT);
            }
        }
    }

    void resendPacket(socket_store_t* socket){
        pack packet;
        transport_packet* payload;

        
        
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
        socket_t fd;
        socket_store_t* socket
        fd = findSocket();
        if(fd != SOCKET_ERROR) {
             socket = &sockets[fd];
            socket->state = CLOSED;
            //receiver
            socket->lastSent = 0;
            socket->lastRcvd = 0;
            socket->nextExpected = 0;
            //sender
            socket->lastWritten = 0;
            socket->lastAck = 0;
            socket->lastSent = 0;
            //socket dress
            socket->src.port = 0;
            socket->src.addr = 0;
            socket->dest.port = 0;
            socket->dest.addr = 0;
            //flow control
            socket->effectiveWindow = 0;
            socket->RTT = 0;

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

    command socket_t Transport.accept(socket_t fd){
        if (fd >= MAX_NUM_OF_SOCKETS || sockets[fd].state != LISTEN){return SOCKET_ERROR;}

        socket_store_t* serverSocket = &sockets[fd]
            socket_t clientFd = findSocket();
            if (clientFd == SOCKET_ERROR){return SOCKET_ERROR}

                socket_store_t* clientSocket = &sockets[clientFd];

                clientSocket->src = serverSocket->src;
                clientSocket->dest = serverSocket->dest;
                clientSocket->state = SYN_RCVD;

                return clientFd;
                // sendPacket(clientSocket);

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

    command error_t release(socket_t fd){
        if(fd >= MAX_NUM_OF_SOCKETS || sockets[fd].state == CLOSED) {
            return FAIL;
        }

        socket_store_t* socket = &sockets[fd];

        if (socket->state != FIN_WAIT && socket->state != CLOSED){return FAIL}

        socket->state = CLOSED;

        socket->lastWritten = 0;
        socket->lastAck = 0;
        socket->lastSent = 0;

        
        socket->lastRead = 0;
        socket->lastRcvd = 0;
        socket->nextExpected = 0;

        //cancel timer 

        return SUCCESS;
    }

    command uint16_t write(socket_t fd, uint8_t *buff, uint16_t bufflen){
        uint8_t index;
        uint16_t i;
        uint16_t availableBuff;
        uint16_t bytesWrite;
        

        if(fd >= MAX_NUM_OF_SOCKETS || sockets[fd].state == CLOSED) {
            return FAIL;
        }

        socket_store_t* socket = &socket[fd]

         availableBuff = SOCKET_BUFFER_SIZE - (socket->lastWritten - socket->lastAck) % SOCKET_BUFFER_SIZE;

        

        if(bufflen < availableBuff){
            bytesWrite = bufflen;
        }
        else{
            bytesWrite = availableBuff;
        }

        for (i = 0; i < bytesWrite; i++){
            index = (socket->lastWritten + i) % SOCKET_BUFFER_SIZE;
            socket->sendBuff[index] = buff[i];
        }

        socket->lastWritten += bytesWrite;

        return bytesWrite;

    }


        

    command uint16_t read(socket_t fd, uint8_t *buff, uint16_t bufflen){
        uint8_t index;
        uint16_t i;
        uint16_t availableBuff;
        uint16_t bytesRead;

        if(fd >= MAX_NUM_OF_SOCKETS || sockets[fd].state == CLOSED) {
            return FAIL;
        }

         socket_store_t* socket = &socket[fd]

         availableBuff = SOCKET_BUFFER_SIZE - (socket->lastRcvd - socket->lastRead) % SOCKET_BUFFER_SIZE;

        

        if(bufflen < availableBuff){
            bytesRead = bufflen;
        }
        else{
            bytesRead = availableBuff;
        }

        for (i = 0; i < bytesRead; i++){
            index = (socket->lastRead + i) % SOCKET_BUFFER_SIZE;
              buff[i] = socket->rcvdBuff[index];
        }

        socket->lastread += bytesRead;

        return bytesRead;

    }



    command error_t Transport.receive(pack* package){
        socket_store_t* socket;
        socket_t fd;
        transport_packet* payload
        if (packet->protocl != PROTOCOL_TRANSPORT){
            return FAIL
        }

            payload = (transport_packet*)packet->payload;

            fd = findSocketByAddr(&payload->src, &payload->desr);
        if(fd == SOCKET_ERROR){
            return FAIL;
        }

         socket = &sockets[fd]

        if (socket == NULL){
            return FAIL;
            // dbg(TRANSPORT_CHANNEL, "")
        }

        updateAdvertWindow(socket);

        switch (socket->state){
            case LISTEN:
                socket_t clientFd;
                ocket_store_t* newSocket
                if(payload->flag & TRANSPORT_SYN){
                    // sendPacket(socket);
                    // socket->state = SYN_RCVD;
                    clientFd =  Transport.accept(fd);
                    if (clientFd == SOCKET_ERROR){return FAIL}
                     newSocket = &sockets[clientFD];

                    sendPacket(newSocket); //SYN-ACK

                    return SUCCESS;
                }
                break;
                // client side 
            case SYN_SENT:
            uint16_t RTT;
                if ((payload->flag & TRANSPORT_SYN) && (payload->flag & TRANSPORT_ACK)){
                    RTT = (call RTTTimer.getNow()) - (socket->sendTimes[payload->seq]);
                    socket->RTT = RTT;
                    socket->state = ESTABLISHED;
                    sendPacket(socket); //Send Ack
                }
                break;
                //serve side
            case SYN_RCVD:
                if(payload->flag & TRANSPORT_ACK){
                    socket->state = ESTABLISHED; //Connection Established
                }
                break;
            case ESTABLISHED:
                if (payload->flag & TRANSPORT_DATA){
                    if(payload->seq == socket->nextExpected){
                        //if the packet was in order (seq == expected seq)
                        uint8_t index = (socket->lastRcvd) % SOCKET_BUFFER_SIZE;
                        socket->rcvdBuff[index] = payload->data[0];
                        socket->lastRcvd++;
                        socket->nextExpected++;

                        return Transport.read(fd);
                    } else if{
                        //Out of Order: Buffer
                         index = (payload->seq) % SOCKET_BUFFER_SIZE;
                         socket->rcvdBuff[index] = payload->data[0];
                    }
                    
                }
            case FIN_WAIT:
                if(payload->flag & TRANSPORT_ACK){
                    // socket->state = CLOSED;
                    return Transport.release(socket);
                }
        }
    }

    event void TimeoutTimer.fired() {
        // gonna handle the  retransmissions here
        uint8_t i;
        socket_store_t* socket;
        for (i = 0; i < MAX_NUM_OF_SOCKETS; i++){
        if(socket->state == SYN_SENT || socket->state == ESTABLISHED || socket->state == FIN_WAIT){
            if(socket->retries < MAX_RETRIES){
                sendPacket(socket);
                socket->retries++
                call TimeoutTimer.startOneShot(TIMEOUT);
            }else{
                socket->state = CLOSED;
             }
            }
        }
    }
    event void RTTTimer.fired(){

    }
}