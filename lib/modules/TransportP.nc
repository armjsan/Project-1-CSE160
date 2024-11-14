#include "../../includes/packet.h"
#include "../../includes/socket.h"
module TransportP{
    provides interface Transport;
         uses interface Timer<TMilli> as timeoutTimer;
}

implementation{
    //Need a timeout timer
    //For sending the ACK and stuff
    socket_store_t socketStore[MAX_NUM_OF_SOCKETS];
    socket_t checkSocket(){
        socket_t fd;
        for (fd = 0; fd < MAX_NUM_OF_SOCKETS; fd++){
            if (socketStore[fd].state == CLOSED){
                socketStore[fd].state = LISTEN;
                socketStore[fd].lastSent = 0;
                socketStore[fd].lastRcvd = 0;
                socketStore[fd].nextExpected = 0;
                return fd;
            }
        }

        return (socket_t) -1;
        

    }

    void sendSockPacket(socket_store_t* socket){
        
        switch (sock->state){
            case SYN_SENT:

            case SYN_RCVD:
                
        }

    }
    command error_t Transport.start(){
        socket_t tempFd;
        for (tempFd = 0; tempFd < MAX_NUM_OF_SOCKETS; tempFd++){
            socketStore[tempFD].state = CLOSED;
            socketStore[tempFD].lastSent = 0;
            socketStore[tempFD].lastRcvd = 0;
            socketStore[tempFD].nextExpected = 0;
        }

        call timeoutTimer.startOneShot(1000); //timer for timeout for SYN_Send & Syn_ACK, etc
        return SUCCESS;


    }
    // set up 
    command error_t Transport.connect(socket_t fd, socket_addr_t *addr){
        socket_store_t* socket = &socketStore[fd]
        if (socket->state == CLOSED){
            socket->dest == *addr;
            socket->state = SYN_SENT;
            sendSockPacket(sock);
        }

    }
    
    command socket_t Transport.accept(socket_t fd){
        socket_store_t* serverSocket = &socketStore[fd]
        if (socket->state == LISTEN){
            socket_t clientFd = checkSocket();
            if (clientFd != (socket_t)-1){
                socket_store_t* clientSocket = &socketStore[clientFd];
                clientSocket->state = SYN_RCVD;
                sendSockPacket(clientSocket);
            }


        }

    }

    //tear down
    command error_t Transport.close(socket_t fd){

    }

    command error_t Transport.release(socket_t fd){

    }
}