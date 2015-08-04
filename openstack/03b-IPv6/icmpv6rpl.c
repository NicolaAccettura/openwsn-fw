#include "opendefs.h"
#include "icmpv6rpl.h"
#include "icmpv6.h"
#include "openserial.h"
#include "openqueue.h"
#include "neighbors.h"
#include "packetfunctions.h"
#include "openrandom.h"
#include "scheduler.h"
#include "idmanager.h"
#include "opentimers.h"
#include "IEEE802154E.h"

//=========================== variables =======================================

icmpv6rpl_vars_t             icmpv6rpl_vars;

//=========================== prototypes ======================================

// DIO-related
void icmpv6rpl_timer_DIO_cb(opentimer_id_t id);
void icmpv6rpl_timer_DIO_task(void);
void sendDIO(void);
// DAO-related
void icmpv6rpl_timer_DAO_cb(opentimer_id_t id);
void icmpv6rpl_timer_DAO_task(void);
void sendDAO(void);

//=========================== public ==========================================

/**
\brief Initialize this module.
*/
void icmpv6rpl_init() {
   uint8_t         dodagid[16];
   uint32_t        dioPeriod;
   uint32_t        daoPeriod;
   
   // retrieve my prefix and EUI64
   memcpy(&dodagid[0],idmanager_getMyID(ADDR_PREFIX)->prefix,8); // prefix
   memcpy(&dodagid[8],idmanager_getMyID(ADDR_64B)->addr_64b,8);  // eui64
   
   //===== reset local variables
   memset(&icmpv6rpl_vars,0,sizeof(icmpv6rpl_vars_t));
   
   //=== admin
   
   icmpv6rpl_vars.busySendingDIO            = FALSE;
   icmpv6rpl_vars.busySendingDAO            = FALSE;
   icmpv6rpl_vars.fDodagidWritten           = 0;
   
   //=== DIO
   
   icmpv6rpl_vars.dio.rplinstanceId         = 0x00;        ///< TODO: put correct value
   icmpv6rpl_vars.dio.verNumb               = 0x00;        ///< TODO: put correct value
   // rank: to be populated upon TX
   icmpv6rpl_vars.dio.rplOptions            = MOP_DIO_A | \
                                              MOP_DIO_B | \
                                              MOP_DIO_C | \
                                              PRF_DIO_A | \
                                              PRF_DIO_B | \
                                              PRF_DIO_C | \
                                              G_DIO ;
   icmpv6rpl_vars.dio.DTSN                  = 0x33;        ///< TODO: put correct value
   icmpv6rpl_vars.dio.flags                 = 0x00;
   icmpv6rpl_vars.dio.reserved              = 0x00;
   memcpy(
      &(icmpv6rpl_vars.dio.DODAGID[0]),
      dodagid,
      sizeof(icmpv6rpl_vars.dio.DODAGID)
   ); // can be replaced later
   
   icmpv6rpl_vars.dioDestination.type = ADDR_128B;
   memcpy(&icmpv6rpl_vars.dioDestination.addr_128b[0],all_routers_multicast,sizeof(all_routers_multicast));
   
   icmpv6rpl_vars.dioPeriod                 = TIMER_DIO_TIMEOUT;
   dioPeriod                                = icmpv6rpl_vars.dioPeriod - 0x80 + (openrandom_get16b()&0xff);
   icmpv6rpl_vars.timerIdDIO                = opentimers_start(
                                                dioPeriod,
                                                TIMER_PERIODIC,
                                                TIME_MS,
                                                icmpv6rpl_timer_DIO_cb
                                             );
   
   //=== DAO
   
   icmpv6rpl_vars.dao.rplinstanceId         = 0x00;        ///< TODO: put correct value
   icmpv6rpl_vars.dao.K_D_flags             = FLAG_DAO_A   | \
                                              FLAG_DAO_B   | \
                                              FLAG_DAO_C   | \
                                              FLAG_DAO_D   | \
                                              FLAG_DAO_E   | \
                                              PRF_DIO_C    | \
                                              FLAG_DAO_F   | \
                                              D_DAO        |
                                              K_DAO;
   icmpv6rpl_vars.dao.reserved              = 0x00;
   icmpv6rpl_vars.dao.DAOSequence           = 0x00;
   memcpy(
      &(icmpv6rpl_vars.dao.DODAGID[0]),
      dodagid,
      sizeof(icmpv6rpl_vars.dao.DODAGID)
   );  // can be replaced later
   
   icmpv6rpl_vars.dao_transit.type          = OPTION_TRANSIT_INFORMATION_TYPE;
   // optionLength: to be populated upon TX
   icmpv6rpl_vars.dao_transit.E_flags       = E_DAO_Transit_Info;
   icmpv6rpl_vars.dao_transit.PathControl   = PC1_B_DAO_Transit_Info;  
   icmpv6rpl_vars.dao_transit.PathSequence  = 0x00; // to be incremented at each TX
   icmpv6rpl_vars.dao_transit.PathLifetime  = 0xAA;
   //target information
   icmpv6rpl_vars.dao_target.type  = OPTION_TARGET_INFORMATION_TYPE;
   icmpv6rpl_vars.dao_target.optionLength  = 0;
   icmpv6rpl_vars.dao_target.flags  = 0;
   icmpv6rpl_vars.dao_target.prefixLength = 0;
   
   icmpv6rpl_vars.daoPeriod                 = TIMER_DAO_TIMEOUT;
   daoPeriod                                = icmpv6rpl_vars.daoPeriod - 0x80 + (openrandom_get16b()&0xff);
   icmpv6rpl_vars.timerIdDAO                = opentimers_start(
                                                daoPeriod,
                                                TIMER_PERIODIC,
                                                TIME_MS,
                                                icmpv6rpl_timer_DAO_cb
                                             );
   
}

void  icmpv6rpl_writeDODAGid(uint8_t* dodagid) {
   
   // write DODAGID to DIO/DAO
   memcpy(
      &(icmpv6rpl_vars.dio.DODAGID[0]),
      dodagid,
      sizeof(icmpv6rpl_vars.dio.DODAGID)
   );
   memcpy(
      &(icmpv6rpl_vars.dao.DODAGID[0]),
      dodagid,
      sizeof(icmpv6rpl_vars.dao.DODAGID)
   );
   
   // remember I got a DODAGID
   icmpv6rpl_vars.fDodagidWritten = 1;
}

uint8_t icmpv6rpl_getRPLIntanceID(){
   return icmpv6rpl_vars.dao.rplinstanceId;
}

/**
\brief Called when DIO/DAO was sent.

\param[in] msg   Pointer to the message just sent.
\param[in] error Outcome of the sending.
*/
void icmpv6rpl_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   bool isDIO;
   
   // take ownership over that packet
   msg->owner = COMPONENT_ICMPv6RPL;
   
   // make sure I created it
   if (msg->creator!=COMPONENT_ICMPv6RPL) {
      openserial_printError(COMPONENT_ICMPv6RPL,ERR_UNEXPECTED_SENDDONE,
                            (errorparameter_t)0,
                            (errorparameter_t)0);
   }
   
   isDIO = msg->l4_isDIO;
   
   // free packet
   openqueue_freePacketBuffer(msg);
   
   // I'm not busy sending anymore
   if (isDIO==TRUE) {
      icmpv6rpl_vars.busySendingDIO = FALSE;
   } else {
      /*if (error == E_FAIL) {
         printf("%02x LOST %02x ASN %02x%04x%04x\n",
            msg->l3_sourceAdd.addr_128b[15],
            idmanager_getMyID(ADDR_64B)->addr_64b[7],
            msg->l2_asn.byte4,
            msg->l2_asn.bytes2and3,
            msg->l2_asn.bytes0and1);
      }*/
      icmpv6rpl_vars.busySendingDAO = FALSE;
   }
}

/**
\brief Called when RPL message received.

\param[in] msg   Pointer to the received message.
*/
void icmpv6rpl_receive(OpenQueueEntry_t* msg) {
   uint8_t      icmpv6code;
   open_addr_t  myPrefix;
   bool         f_newDODAGID;
   
   // take ownership
   msg->owner      = COMPONENT_ICMPv6RPL;
   
   // retrieve ICMPv6 code
   icmpv6code      = (((ICMPv6_ht*)(msg->payload))->code);
   
   // toss ICMPv6 header
   packetfunctions_tossHeader(msg,sizeof(ICMPv6_ht));
   
   // handle message
   switch (icmpv6code) {
      
      case IANA_ICMPv6_RPL_DIO:
         if (idmanager_getIsDAGroot()==TRUE) {
            // stop here if I'm in the DAG root
            break; // break, don't return
         }
         
         // register whether the DIO owns a new DODAGID
         f_newDODAGID = (memcmp(
            &(((icmpv6rpl_dio_ht*)(msg->payload))->DODAGID[0]),
            &(icmpv6rpl_vars.dio.DODAGID[0]),
            sizeof(icmpv6rpl_vars.dio.DODAGID)
            )!=0);
         
         // update neighbor table
         neighbors_indicateRxDIO(msg,&f_newDODAGID);
         
         if (f_newDODAGID) {
            // write DODAGID in DIO and DAO
            icmpv6rpl_writeDODAGid(&(((icmpv6rpl_dio_ht*)(msg->payload))->DODAGID[0]));
            
            // update my prefix
            myPrefix.type = ADDR_PREFIX;
            memcpy(
               myPrefix.prefix,
               &((icmpv6rpl_dio_ht*)(msg->payload))->DODAGID[0],
               sizeof(myPrefix.prefix)
            );
            idmanager_setMyID(&myPrefix);
         }
         break;
      
      case IANA_ICMPv6_RPL_DAO:
         // this should never happen
         openserial_printCritical(COMPONENT_ICMPv6RPL,ERR_UNEXPECTED_DAO,
                               (errorparameter_t)0,
                               (errorparameter_t)0);
         break;
      
      default:
         // this should never happen
         openserial_printCritical(COMPONENT_ICMPv6RPL,ERR_MSG_UNKNOWN_TYPE,
                               (errorparameter_t)icmpv6code,
                               (errorparameter_t)0);
         break;
      
   }
   
   // free message
   openqueue_freePacketBuffer(msg);
}

//=========================== private =========================================

//===== DIO-related

/**
\brief DIO timer callback function.

\note This function is executed in interrupt context, and should only push a 
   task.
*/
void icmpv6rpl_timer_DIO_cb(opentimer_id_t id) {
   scheduler_push_task(icmpv6rpl_timer_DIO_task,TASKPRIO_RPL);
}

/**
\brief Handler for DIO timer event.

\note This function is executed in task context, called by the scheduler.
*/
void icmpv6rpl_timer_DIO_task() {
   uint32_t        dioPeriod;
   // send DIO
   sendDIO();
   
   // arm the DIO timer with this new value
   dioPeriod = icmpv6rpl_vars.dioPeriod - 0x80 + (openrandom_get16b()&0xff);
   opentimers_setPeriod(
      icmpv6rpl_vars.timerIdDIO,
      TIME_MS,
      dioPeriod
   );
}

/**
\brief Prepare and a send a RPL DIO.
*/
void sendDIO() {
   OpenQueueEntry_t*    msg;
   
   // stop if I'm not sync'ed
   if (ieee154e_isSynch()==FALSE) {
      
      // remove packets genereted by this module (DIO and DAO) from openqueue
      openqueue_removeAllCreatedBy(COMPONENT_ICMPv6RPL);
      
      // I'm not busy sending a DIO/DAO
      icmpv6rpl_vars.busySendingDIO = FALSE;
      icmpv6rpl_vars.busySendingDAO = FALSE;
      
      // stop here
      return;
   }
   
   // do not send DIO if I have the default DAG rank
   if (
         (neighbors_getMyLowestDAGrank()==DEFAULTDAGRANK)
         &&
         (neighbors_getMyDAGrank()==DEFAULTDAGRANK)
      ) {
      return;
   }
   
   // do not send DIO if I'm already busy sending
   if (icmpv6rpl_vars.busySendingDIO==TRUE) {
      return;
   }
   
   // if you get here, all good to send a DIO
   
   // reserve a free packet buffer for DIO
   msg = openqueue_getFreePacketBuffer(COMPONENT_ICMPv6RPL);
   if (msg==NULL) {
      openserial_printError(COMPONENT_ICMPv6RPL,ERR_NO_FREE_PACKET_BUFFER,
                            (errorparameter_t)0,
                            (errorparameter_t)0);
      
      return;
   }
   
   // take ownership
   msg->creator                             = COMPONENT_ICMPv6RPL;
   msg->owner                               = COMPONENT_ICMPv6RPL;
   
   // set transport information
   msg->l4_protocol                         = IANA_ICMPv6;
   msg->l4_sourcePortORicmpv6Type           = IANA_ICMPv6_RPL;
   
   // mark this packet as DIO for unlocking busySendingDIO while executing sendDone
   msg->l4_isDIO = TRUE;
   
   // set DIO destination
   memcpy(&(msg->l3_destinationAdd),&icmpv6rpl_vars.dioDestination,sizeof(open_addr_t));
   
   //===== DIO payload
   // note: DIO is already mostly populated
   icmpv6rpl_vars.dio.rank                  = neighbors_getMyDAGrank();
   neighbors_setAdvertizedDAGrank();
   packetfunctions_reserveHeaderSize(msg,sizeof(icmpv6rpl_dio_ht));
   memcpy(
      ((icmpv6rpl_dio_ht*)(msg->payload)),
      &(icmpv6rpl_vars.dio),
      sizeof(icmpv6rpl_dio_ht)
   );
   
   // reverse the rank bytes order in Big Endian
   *(msg->payload+2) = (icmpv6rpl_vars.dio.rank >> 8) & 0xFF;
   *(msg->payload+3) = icmpv6rpl_vars.dio.rank        & 0xFF;
   
   //===== ICMPv6 header
   packetfunctions_reserveHeaderSize(msg,sizeof(ICMPv6_ht));
   ((ICMPv6_ht*)(msg->payload))->type       = msg->l4_sourcePortORicmpv6Type;
   ((ICMPv6_ht*)(msg->payload))->code       = IANA_ICMPv6_RPL_DIO;
   packetfunctions_calculateChecksum(msg,(uint8_t*)&(((ICMPv6_ht*)(msg->payload))->checksum));//call last
   
   //send
   if (icmpv6_send(msg)!=E_SUCCESS) {
      openqueue_freePacketBuffer(msg);
   } else {
      // I'm now busy sending
      icmpv6rpl_vars.busySendingDIO = TRUE; 
   }
}

//===== DAO-related

/**
\brief DAO timer callback function.

\note This function is executed in interrupt context, and should only push a
   task.
*/
void icmpv6rpl_timer_DAO_cb(opentimer_id_t id) {
   scheduler_push_task(icmpv6rpl_timer_DAO_task,TASKPRIO_RPL);
}

/**
\brief Handler for DAO timer event.

\note This function is executed in task context, called by the scheduler.
*/
void icmpv6rpl_timer_DAO_task() {
   uint32_t        daoPeriod;
   
   // send DAO
   sendDAO();
   
   // arm the DAO timer with this new value
   daoPeriod = icmpv6rpl_vars.daoPeriod - 0x80 + (openrandom_get16b()&0xff);
   opentimers_setPeriod(
      icmpv6rpl_vars.timerIdDAO,
      TIME_MS,
      daoPeriod
   );
}

/**
\brief Prepare and a send a RPL DAO.
*/
void sendDAO() {
   OpenQueueEntry_t*    msg;                // pointer to DAO messages
   uint8_t              parentIdx;             // running parent index
   uint8_t              numTransitParents;  // the number of parents indicated in transit option
   open_addr_t         address;
   uint8_t              asn[5];
   
   if (ieee154e_isSynch()==FALSE) {
      // I'm not sync'ed 
      
      // delete packets genereted by this module (DIO and DAO) from openqueue
      openqueue_removeAllCreatedBy(COMPONENT_ICMPv6RPL);
      
      // I'm not busy sending a DIO/DAO
      icmpv6rpl_vars.busySendingDIO = FALSE;
      icmpv6rpl_vars.busySendingDAO = FALSE;
      
      // stop here
      return;
   }
   
   // dont' send a DAO if you're the DAG root
   if (idmanager_getIsDAGroot()==TRUE) {
      return;
   }
   
   // dont' send a DAO if you did not acquire a DAGrank
   if (neighbors_getMyDAGrank()==DEFAULTDAGRANK) {
       return;
   }
   
   // dont' send a DAO if you did not send at least a DIO
   if (neighbors_getMyLowestDAGrank()==DEFAULTDAGRANK) {
       return;
   }
   
   // dont' send a DAO if you're still busy sending the previous one
   if (icmpv6rpl_vars.busySendingDAO==TRUE) {
      return;
   }
   
   // if you get here, you start construct DAO
   
   // reserve a free packet buffer for DAO
   msg = openqueue_getFreePacketBuffer(COMPONENT_ICMPv6RPL);
   if (msg==NULL) {
      openserial_printError(COMPONENT_ICMPv6RPL,ERR_NO_FREE_PACKET_BUFFER,
                            (errorparameter_t)0,
                            (errorparameter_t)0);
      return;
   }
   
   // take ownership
   msg->creator                             = COMPONENT_ICMPv6RPL;
   msg->owner                               = COMPONENT_ICMPv6RPL;
   
   // set transport information
   msg->l4_protocol                         = IANA_ICMPv6;
   msg->l4_sourcePortORicmpv6Type           = IANA_ICMPv6_RPL;
   
   // set DAO destination
   msg->l3_destinationAdd.type=ADDR_128B;
   memcpy(msg->l3_destinationAdd.addr_128b,icmpv6rpl_vars.dio.DODAGID,sizeof(icmpv6rpl_vars.dio.DODAGID));
   
   //===== fill in packet
   
   numTransitParents=0;
   for (parentIdx=0;parentIdx<PARENTSETSIZE;parentIdx++) {
      if (neighbors_getParentByIndex(&address, parentIdx)==TRUE) {
         packetfunctions_writeAddress(msg,&address,OW_BIG_ENDIAN);
         icmpv6rpl_vars.dao_transit.optionLength  = LENGTH_ADDR64b + sizeof(icmpv6rpl_dao_transit_ht)-2;
         if (parentIdx==0) {
            icmpv6rpl_vars.dao_transit.PathControl=PC1_B_DAO_Transit_Info; //todo. this is to set the preference of this parent.
         } else {
            icmpv6rpl_vars.dao_transit.PathControl=PC2_B_DAO_Transit_Info; //todo. this is to set the preference of this parent.
         }
         icmpv6rpl_vars.dao_transit.type=OPTION_TRANSIT_INFORMATION_TYPE;
         // write transit info in packet
         packetfunctions_reserveHeaderSize(msg,sizeof(icmpv6rpl_dao_transit_ht));
         memcpy(
                ((icmpv6rpl_dao_transit_ht*)(msg->payload)),
                &(icmpv6rpl_vars.dao_transit),
                sizeof(icmpv6rpl_dao_transit_ht)
         );
         numTransitParents++;
      } else {
         break;
      }
   }
   
   // stop here if no parents found
   if (numTransitParents==0) {
      openqueue_freePacketBuffer(msg);
      return;
   }
   
   icmpv6rpl_vars.dao_transit.PathSequence++; //increment path sequence.
   // if you get here, you will send a DAO
   
   
   //=== DAO header
   packetfunctions_reserveHeaderSize(msg,sizeof(icmpv6rpl_dao_ht));
   memcpy(
      ((icmpv6rpl_dao_ht*)(msg->payload)),
      &(icmpv6rpl_vars.dao),
      sizeof(icmpv6rpl_dao_ht)
   );
   
   //=== ICMPv6 header
   packetfunctions_reserveHeaderSize(msg,sizeof(ICMPv6_ht));
   ((ICMPv6_ht*)(msg->payload))->type       = msg->l4_sourcePortORicmpv6Type;
   ((ICMPv6_ht*)(msg->payload))->code       = IANA_ICMPv6_RPL_DAO;
   packetfunctions_calculateChecksum(msg,(uint8_t*)&(((ICMPv6_ht*)(msg->payload))->checksum)); //call last
   
   //===== send
   if (icmpv6_send(msg)==E_SUCCESS) {
      ieee154e_getAsn(asn);
      /*printf("%02x SEND %02x ASN %02x%02x%02x%02x%02x\n",
         idmanager_getMyID(ADDR_64B)->addr_64b[7],
         idmanager_getMyID(ADDR_64B)->addr_64b[7],
         asn[4],asn[3],asn[2],asn[1],asn[0]);*/
      icmpv6rpl_vars.busySendingDAO = TRUE;
   } else {
      openqueue_freePacketBuffer(msg);
   }
}

void icmpv6rpl_setDIOPeriod(uint16_t dioPeriod){
   uint32_t        dioPeriodRandom;
   
   icmpv6rpl_vars.dioPeriod = dioPeriod;
   dioPeriodRandom = icmpv6rpl_vars.dioPeriod - 0x80 + (openrandom_get16b()&0xff);
   opentimers_setPeriod(
       icmpv6rpl_vars.timerIdDIO,
       TIME_MS,
       dioPeriodRandom
   );
}

void icmpv6rpl_setDAOPeriod(uint16_t daoPeriod){
   uint32_t        daoPeriodRandom;
   
   icmpv6rpl_vars.daoPeriod = daoPeriod;
   daoPeriodRandom = icmpv6rpl_vars.daoPeriod - 0x80 + (openrandom_get16b()&0xff);
   opentimers_setPeriod(
       icmpv6rpl_vars.timerIdDAO,
       TIME_MS,
       daoPeriodRandom
   );
}
