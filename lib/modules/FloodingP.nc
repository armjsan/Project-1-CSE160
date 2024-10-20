#include "../../includes/packet.h"
#include "../../includes/channels.h"
#include "../../includes/NeighborTable.h"
#include "../../includes/sendInfo.h"
#include "../../includes/am_types.h"
#include "../../includes/protocol.h"

module FloodingP {
     provides interface Flooding;

    provides interface SimpleSend as Flooder;

    uses interface SimpleSend as Sender;

    uses interface Receive as Receiver;

    uses interface NeighborDiscovery as Neigh;

    


}

implementation {
    pack sendFlood;
    uint16_t sequenceNum = 0;

     neighbor_t neighborTable[MAX_NEIGHBORS]; 

    event void Neigh.done(){

    }
    command void Flooding.pass() {
        // Dummy implementation
        // uint8_t i;
        // for (i = 0; i < MAX_NEIGHBORS; i++){
        //     neighborTable[i] = neighborDiscovery.getNeighbor()
        // }
    }

    command error_t Flooding.start(){
        dbg(FLOODING_CHANNEL, "FLOODING STARTED\n");
        call Neigh.getNeighbor(neighborTable); //I want to get neighbor table to use 

        sendFlood.src = TOS_NODE_ID;
        sendFlood.seq = sequenceNum;
        sendFlood.TTL = 20;
        sendFlood.type = TOS_NODE_ID;
        sendFlood.protocol = PROTOCOL_PING;
         sendFlood.fdest = neighborTable[10].nodeID;
        memcpy(sendFlood.payload, "Flooding message", 20);
        return SUCCESS;
    }

    

    command error_t Flooder.send(pack msg, uint16_t dest){ //I want to send from the FLood SRC the 
        uint8_t nid;
        uint8_t i = 0;

        nid = neighborTable[i].neighborID;

        if(call Sender.send(sendFlood, nid) == SUCCESS){
         dbg("Flooding", "Unicast Sent Sucessfully\n");
        }
        else{
        dbg("Flooding", "Unicast Sent Failed\n");
        }
    }
        // pack acK;
    event message_t* Receiver.receive(message_t* msg, void* payload, uint8_t len){
        
        uint8_t i;
        pack* myMsg = (pack*) payload;
        dbg(FLOODING_CHANNEL, "Receive Flood Start\n");
        for ( i = 0; i < MAX_NEIGHBORS; i++){
            if (myMsg->TTL == 0){ // if its the final destination
                //right now return msg but actuality, we wantv to send an acklowledgement
                return msg;
            }
            else if(myMsg->src == neighborTable[i].nodeID && neighborTable[i].isActive  == INACTIVE){
                return msg;
            }
            else if (myMsg->fdest == neighborTable[i].neighborID){
                pack acK;
                acK.src = neighborTable[i].neighborID;
                memcpy(acK.payload, "Acknowleding", 20);


                call Sender.send(acK, myMsg->type);
                
            }
            else{
                neighborFlood(neighborTable[i].nodeID);
                dbg(FLOODING_CHANNEL, "neighborFlood Start\n");
            }


        }
        
    }

    void neighborFlood(uint8_t nodeID){
        //Create a function that finds the neighbors
        uint8_t i;
        for ( i = 0; i < MAX_NEIGHBORS; i++){
            if (neighborTable[i].nodeID == nodeID){
                call Sender.send(sendFlood, neighborTable[i].neighborID);
            }
        }







    }







}
