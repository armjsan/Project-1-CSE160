/*
 * ANDES Lab - University of California, Merced
 * This class provides the basic functions of a network node.
 *
 * @author UCM ANDES Lab
 * @date   2013/09/03
 *
 */
#include <Timer.h>
#include "includes/command.h"
#include "includes/packet.h"
#include "includes/CommandMsg.h"
#include "includes/sendInfo.h"
#include "includes/channels.h"

module Node{
   uses interface Boot;

   uses interface SplitControl as AMControl;
   uses interface Receive;

   uses interface SimpleSend as Sender;

   uses interface CommandHandler;

   uses interface NeighborDiscovery; 
   uses interface Flooding; 

   uses interface LinkState;
}

implementation{
   pack sendPackage;

   // Prototypes
      void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t Protocol, uint16_t seq, uint8_t *payload, uint8_t length);

   event void Boot.booted(){
      call AMControl.start();

      dbg(GENERAL_CHANNEL, "Booted\n");

      // neighbor discovery start 
      if (call NeighborDiscovery.start() == SUCCESS) { 
         dbg("NeighborDiscovery", "NeighborDiscovery start command was successful.\n");
      } 
      else {
        dbg("NeighborDiscovery", "NeighborDiscovery start command failed.\n");
      }

      if (call Flooding.start() == SUCCESS) { 
         dbg("Flooding", "Flooding start command was successful.\n");
      } 
      else {
        dbg("Flooding", "Flooding start command failed.\n");
      }
      
   }

   event void AMControl.startDone(error_t err){
      if(err == SUCCESS){
         dbg(GENERAL_CHANNEL, "Radio On\n");
      }else{
         //Retry until successful
         call AMControl.start();
      }
   }

   event void AMControl.stopDone(error_t err){}

   event void NeighborDiscovery.done(){
      dbg(GENERAL_CHANNEL, "Neighbor Discovery DONE\n");
      // LinkState.floodLSA();
   }

   // event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){
   //    dbg(GENERAL_CHANNEL, "Packet Received\n");
   //    if(len==sizeof(pack)){
   //       // to get packet structure 
   //       pack* myMsg=(pack*) payload;
   //       dbg(GENERAL_CHANNEL, "Package Payload: %s\n", myMsg->payload);
   //       return msg;
   //    }
   //    dbg(GENERAL_CHANNEL, "Unknown Packet Type %d\n", len);
   //    return msg;
   // }

   // ------ Neighbor Discovery ---------- // 
   // Received Request packet
   // Send reply packet 
   event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
      pack* myMsg = (pack*) payload; 
      if (myMsg->type == TYPE_REQUEST) {
         dbg(GENERAL_CHANNEL, "Received package payload %s from %d\n", myMsg -> payload, myMsg->src);

         // Send reply message 
         sendPackage.src = TOS_NODE_ID; 
         sendPackage.dest = myMsg->src; 
         sendPackage.type = TYPE_REPLY; 
         sendPackage.protocol = PROTOCOL_PING; 
         memcpy(sendPackage.payload, "reply", 5); 

         if (call Sender.send(sendPackage, TOS_NODE_ID) == SUCCESS) { 
            dbg(GENERAL_CHANNEL, "reply message sent successfully from %d\n", myMsg->src); 
         }
         else { 
            // Link might be INACTIVE
            dbg(GENERAL_CHANNEL, "reply message failed to send, node may be in Active\n"); 
         }


         call NeighborDiscovery.handleNeighbor(myMsg->src, 100); // Call a NeighborDiscovery command


         
      }
      return msg;
   }

   // signal NeighborDiscovery.done();
   

   event void CommandHandler.ping(uint16_t destination, uint8_t *payload){
      dbg(GENERAL_CHANNEL, "PING EVENT \n");
      makePack(&sendPackage, TOS_NODE_ID, destination, 0, 0, 0, payload, PACKET_MAX_PAYLOAD_SIZE);
      call Sender.send(sendPackage, destination);
   }

   event void CommandHandler.printNeighbors(){}

   event void CommandHandler.printRouteTable(){}

   event void CommandHandler.printLinkState(){}

   event void CommandHandler.printDistanceVector(){}

   event void CommandHandler.setTestServer(){}

   event void CommandHandler.setTestClient(){}

   event void CommandHandler.setAppServer(){}

   event void CommandHandler.setAppClient(){}

   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t protocol, uint16_t seq, uint8_t* payload,  uint8_t length){
      Package->src = src;
      Package->dest = dest;
      Package->TTL = TTL;
      Package->seq = seq;
      Package->protocol = protocol;
      // Package->type = type;
      // Package-> fdest = fdest;
      memcpy(Package->payload, payload, length);
   }
}
