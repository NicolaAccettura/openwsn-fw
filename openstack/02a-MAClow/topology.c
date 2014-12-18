#include "opendefs.h"
#include "topology.h"
#include "idmanager.h"

//=========================== defines =========================================

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== public ==========================================

/**
\brief Force a topology.

This function is used to force a certain topology, by hard-coding the list of
acceptable neighbors for a given mote. This function is invoked each time a
packet is received. If it returns FALSE, the packet is silently dropped, as if
it were never received. If it returns TRUE, the packet is accepted.

Typically, filtering packets is done by analyzing the IEEE802.15.4 header. An
example body for this function which forces a topology is:

   switch (idmanager_getMyID(ADDR_64B)->addr_64b[7]) {
      case TOPOLOGY_MOTE1:
         if (ieee802514_header->src.addr_64b[7]==TOPOLOGY_MOTE2) {
            returnVal=TRUE;
         } else {
            returnVal=FALSE;
         }
         break;
      case TOPOLOGY_MOTE2:
         if (ieee802514_header->src.addr_64b[7]==TOPOLOGY_MOTE1 ||
             ieee802514_header->src.addr_64b[7]==TOPOLOGY_MOTE3) {
            returnVal=TRUE;
         } else {
            returnVal=FALSE;
         }
         break;
      default:
         returnVal=TRUE;
   }
   return returnVal;

By default, however, the function should return TRUE to *not* force any
topology.

\param[in] ieee802514_header The parsed IEEE802.15.4 MAC header.

\return TRUE if the packet can be received.
\return FALSE if the packet should be silently dropped.
*/
bool topology_isAcceptablePacket(ieee802154_header_iht* ieee802514_header) {
#ifdef FORCETOPOLOGY
   bool returnVal;
   
   returnVal=FALSE;
   switch ((idmanager_getMyID(ADDR_64B)->addr_64b[6]<<8) | idmanager_getMyID(ADDR_64B)->addr_64b[7]) {
      case 0xf185:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x8d59 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90dd
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x8d59:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0xf185 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x8d5c ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x9098
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x8d5c:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x8d59 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x8d70 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x9080
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x8d70:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x8d5c ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x8d73 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x8d76
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x8d73:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x8d70
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x8d76:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x8d70
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x9080:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x8d5c ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x908b ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x9096
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x908b:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x9080
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x9096:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x9080
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x9098:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x8d59 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90a5 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90ad
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x90a5:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x9098 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90a6 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90a8
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x90a6:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90a5
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x90a8:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90a5
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x90ad:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x9098 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90d1 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90d5
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x90d1:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90ad
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x90d5:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90ad
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x90dd:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0xf185 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90e4 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x94bc
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x90e4:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90dd ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90f0 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x94ad
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x90f0:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90e4 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90f7 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x9483
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x90f7:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90f0
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x9483:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90f0
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x94ad:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90e4 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x94b4 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x94b8
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x94b4:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x94ad
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x94b8:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x94ad
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x94bc:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x90dd ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x94be ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x94e3
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x94be:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x94bc ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x94c6 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x94d5
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x94c6:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x94be
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x94d5:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x94be
            ) {
            returnVal=TRUE;
         }
         break;
      case 0x94e3:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x94bc ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0xede6 ||
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0xedfa
            ) {
            returnVal=TRUE;
         }
         break;
      case 0xede6:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x94e3
            ) {
            returnVal=TRUE;
         }
         break;
      case 0xedfa:
         if (
            ((ieee802514_header->src.addr_64b[6]<<8) | ieee802514_header->src.addr_64b[7])==0x94e3
            ) {
            returnVal=TRUE;
         }
         break;
   }
   return returnVal;
#else
   return TRUE;
#endif
}

//=========================== private =========================================
