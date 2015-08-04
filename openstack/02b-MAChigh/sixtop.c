#include "opendefs.h"
#include "sixtop.h"
#include "openserial.h"
#include "openqueue.h"
#include "neighbors.h"
#include "IEEE802154E.h"
#include "iphc.h"
#include "otf.h"
#include "packetfunctions.h"
#include "openrandom.h"
#include "scheduler.h"
#include "opentimers.h"
#include "debugpins.h"
#include "leds.h"
#include "processIE.h"
#include "IEEE802154.h"
#include "IEEE802154_security.h"
#include "idmanager.h"
#include "schedule.h"

//=========================== variables =======================================

sixtop_vars_t sixtop_vars;

//=========================== prototypes ======================================

// send internal
owerror_t     sixtop_send_internal(
   OpenQueueEntry_t*    msg,
   bool                 payloadIEPresent
);

// timer interrupt callbacks
void          sixtop_maintenance_timer_cb(opentimer_id_t id);
void          sixtop_timeout_timer_cb(opentimer_id_t id);

//=== EB/KA/maintainance task

void          timer_sixtop_management_fired(void);
void          sixtop_sendEB(void);
void          sixtop_sendKA(void);
void          sixtop_maintaining(void);

//=== six2six task

void          timer_sixtop_six2six_timeout_fired(void);
void          sixtop_six2six_sendDone(
   OpenQueueEntry_t*    msg,
   owerror_t            error
);
bool          sixtop_processIEs(
   OpenQueueEntry_t*    pkt,
   uint16_t*            lenIE
);
void          sixtop_notifyReceiveCommand(
   opcode_IE_ht*        opcode_ie, 
   bandwidth_IE_ht*     bandwidth_ie, 
   schedule_IE_ht*      schedule_ie,
   open_addr_t*         addr
);
owerror_t     sixtop_sendCommand(void);

//=== helper functions
owerror_t     sixtop_removeCells_private(
   open_addr_t*         neighbor,
   uint16_t             numCells,
   six2six_handler_t    handler
);
bool          sixtop_candidateAddCellList(void);
bool          sixtop_candidateRemoveCellList(
   open_addr_t*         neighbor
);
void          sixtop_addCellsByState(void);
void          sixtop_removeCellsByState(
   schedule_IE_ht*      schedule_ie,
   open_addr_t*         neighbor
);
bool          sixtop_areAvailableCellsToBeScheduled(void);
void          sixtop_resetState(owerror_t error);

//=========================== public ==========================================

void sixtop_init() {
   
   memset(&sixtop_vars,0,sizeof(sixtop_vars_t));
   
   sixtop_vars.periodMaintenance  = 872 +(openrandom_get16b()&0xff);
   sixtop_vars.busySendingEB      = FALSE;
   sixtop_vars.dsn                = 0;
   sixtop_vars.mgtTaskCounter     = 0;
   sixtop_vars.kaPeriod           = MAXKAPERIOD;
   sixtop_vars.ebPeriod           = EBPERIOD;
   sixtop_vars.six2six_state      = SIX_IDLE;
   sixtop_vars.handler            = SIX_HANDLER_NONE;
   
   sixtop_vars.maintenanceTimerId = opentimers_start(
      sixtop_vars.periodMaintenance,
      TIMER_PERIODIC,
      TIME_MS,
      sixtop_maintenance_timer_cb
   );
   
   sixtop_vars.timeoutTimerId     = opentimers_start(
      SIX2SIX_TIMEOUT_MS,
      TIMER_ONESHOT,
      TIME_MS,
      sixtop_timeout_timer_cb
   );
}

void sixtop_setKaPeriod(uint16_t kaPeriod) {
   if(kaPeriod > MAXKAPERIOD) {
      sixtop_vars.kaPeriod = MAXKAPERIOD;
   } else {
      sixtop_vars.kaPeriod = kaPeriod;
   } 
}

void sixtop_setEBPeriod(uint8_t ebPeriod) {
   if(ebPeriod < SIXTOP_MINIMAL_EBPERIOD) {
      sixtop_vars.ebPeriod = SIXTOP_MINIMAL_EBPERIOD;
   } else {
      sixtop_vars.ebPeriod = ebPeriod;
   } 
}

//======= scheduling

owerror_t sixtop_addCells(open_addr_t* neighbor, uint16_t numCells, six2six_handler_t handler){
   bool              outcome;
   
   if (neighbors_getMyLowestDAGrank()==DEFAULTDAGRANK) {
      return E_FAIL;
   }
   
   // filter parameters
   if(sixtop_vars.six2six_state!=SIX_IDLE){
      return E_FAIL;
   }
   
   if ((neighbor == NULL) || (neighbor->type == ADDR_NONE)) {
      return E_FAIL;
   }
   
   if (numCells>SCHEDULEIEMAXNUMCELLS) {
      return E_FAIL;
   }
   
   // build opcode_ie
   sixtop_vars.opcode_ie.opcode = SIXTOP_SOFT_CELL_REQUEST;
   sixtop_vars.opcode_ie.seqNumber = sixtop_vars.seqNumber;
   
   // build bandwidth_ie
   sixtop_vars.bandwidth_ie.slotframeID = schedule_getFrameHandle();
   sixtop_vars.bandwidth_ie.numOfLinks = numCells;
   
   // generate candidate cell list and build schedule_ie
   outcome = sixtop_candidateAddCellList();
   if (outcome == FALSE) {
      sixtop_resetState(E_FAIL);
      return E_FAIL;
   }
   
   // store neighbor to match response
   memcpy(&(sixtop_vars.currentNeighbor),neighbor,sizeof(open_addr_t));
   
   if (sixtop_sendCommand() == E_FAIL) {
      sixtop_resetState(E_FAIL);
      return E_FAIL;
   }
   
   // update state
   sixtop_vars.six2six_state = SIX_WAIT_ADDREQUEST_SENDDONE;
   sixtop_vars.handler = handler;
   return E_SUCCESS;
}

owerror_t sixtop_removeCells(open_addr_t* neighbor, uint16_t numCells, six2six_handler_t handler){
   
   if (neighbors_getMyLowestDAGrank()==DEFAULTDAGRANK) {
      return E_FAIL;
   }
   
   // filter parameters
   if (sixtop_vars.six2six_state!=SIX_IDLE){
      return E_FAIL;
   }
   
   if ((neighbor == NULL) || (neighbor->type == ADDR_NONE)) {
      return E_FAIL;
   }
   
   if (numCells>SCHEDULEIEMAXNUMCELLS) {
      return E_FAIL;
   }
   
   return sixtop_removeCells_private(neighbor, numCells, handler);
}

//======= from upper layer

owerror_t sixtop_send(OpenQueueEntry_t *msg) {
   
   // set metadata
   msg->owner        = COMPONENT_SIXTOP;
   msg->l2_frameType = IEEE154_TYPE_DATA;
   
   
   // set l2-security attributes
   msg->l2_securityLevel   = IEEE802154_SECURITY_LEVEL;
   msg->l2_keyIdMode       = IEEE802154_SECURITY_KEYIDMODE; 
   msg->l2_keyIndex        = IEEE802154_SECURITY_K2_KEY_INDEX;
   
   if (msg->l2_payloadIEpresent == FALSE) {
      return sixtop_send_internal(
         msg,
         FALSE
      );
   } else {
      return sixtop_send_internal(
         msg,
         TRUE
      );
   }
}

//======= from lower layer

void task_sixtopNotifSendDone() {
   OpenQueueEntry_t* msg;
   bool              succesfullTx;
   
   // get recently-sent packet from openqueue
   msg = openqueue_sixtopGetSentPacket();
   if (msg==NULL) {
      openserial_printCritical(
         COMPONENT_SIXTOP,
         ERR_NO_SENT_PACKET,
         (errorparameter_t)0,
         (errorparameter_t)0
      );
      return;
   }
   
   // take ownership
   msg->owner = COMPONENT_SIXTOP;
   
   // update neighbor statistics
   succesfullTx = FALSE;
   if (msg->l2_sendDoneError==E_SUCCESS) {
      succesfullTx = TRUE;
   }
   neighbors_indicateTx(&(msg->l2_nextORpreviousHop),succesfullTx,&msg->l2_asn);
   
   if ((msg->l2_retriesLeft > 0) && (succesfullTx == FALSE)) {
      // return packet to the virtual COMPONENT_SIXTOP_TO_IEEE802154E component
      msg->owner = COMPONENT_SIXTOP_TO_IEEE802154E;
      return;
   }
   /*if (
         (msg->l2_sendDoneError==E_FAIL)
         &&
         (msg->creator != COMPONENT_SIXTOP_RES)
         &&
         (schedule_getNumTxCellsToNeighbor(&(msg->l2_nextORpreviousHop))==0)
      ) {
      // call sixtop
      sixtop_addCells(
         &(msg->l2_nextORpreviousHop),
         1
      );
   }*/
   
   // send the packet to where it belongs
   switch (msg->creator) {
      
      case COMPONENT_SIXTOP:
         if (msg->l2_frameType==IEEE154_TYPE_BEACON) {
            // this is a EB
            
            // not busy sending EB anymore
            sixtop_vars.busySendingEB = FALSE;
         }
         
         // discard packets
         openqueue_freePacketBuffer(msg);
         
         // restart a random timer
         sixtop_vars.periodMaintenance = 872+(openrandom_get16b()&0xff);
         opentimers_setPeriod(
            sixtop_vars.maintenanceTimerId,
            TIME_MS,
            sixtop_vars.periodMaintenance
         );
         break;
      
      case COMPONENT_SIXTOP_RES:
         sixtop_six2six_sendDone(msg,msg->l2_sendDoneError);
         break;
      
      default:
         // send the rest up the stack
         iphc_sendDone(msg,msg->l2_sendDoneError);
         break;
   }
}

void task_sixtopNotifReceive() {
   OpenQueueEntry_t* msg;
   uint16_t          lenIE;
   
   // get received packet from openqueue
   msg = openqueue_sixtopGetReceivedPacket();
   if (msg==NULL) {
      openserial_printCritical(
         COMPONENT_SIXTOP,
         ERR_NO_RECEIVED_PACKET,
         (errorparameter_t)0,
         (errorparameter_t)0
      );
      return;
   }
   
   // take ownership
   msg->owner = COMPONENT_SIXTOP;
   
   // process the header IEs
   lenIE=0;
   if(
         msg->l2_frameType              == IEEE154_TYPE_DATA  &&
         msg->l2_payloadIEpresent       == TRUE               &&
         sixtop_processIEs(msg, &lenIE) == FALSE
      ) {
      // free the packet's RAM memory
      openqueue_freePacketBuffer(msg);
      //log error
      return;
   }
   
   // toss the header IEs
   packetfunctions_tossHeader(msg,lenIE);
   
   // update neighbor statistics
   neighbors_indicateRx(
      &(msg->l2_nextORpreviousHop),
      msg->l1_rssi,
      &msg->l2_asn,
      msg->l2_joinPriorityPresent,
      msg->l2_joinPriority
   );
   
   // reset it to avoid race conditions with this var.
   msg->l2_joinPriorityPresent = FALSE; 
   
   // send the packet up the stack, if it qualifies
   switch (msg->l2_frameType) {
      case IEEE154_TYPE_BEACON:
      case IEEE154_TYPE_DATA:
      case IEEE154_TYPE_CMD:
         if (msg->length>0) {
            // send to upper layer
            iphc_receive(msg);
         } else {
            // free up the RAM
            openqueue_freePacketBuffer(msg);
         }
         break;
      case IEEE154_TYPE_ACK:
      default:
         // free the packet's RAM memory
         openqueue_freePacketBuffer(msg);
         // log the error
         openserial_printError(
            COMPONENT_SIXTOP,
            ERR_MSG_UNKNOWN_TYPE,
            (errorparameter_t)msg->l2_frameType,
            (errorparameter_t)0
         );
         break;
   }
}

void task_sixtopNotifSlotframe() {
   otf_housekeeping();
}

//======= debugging

/**
\brief Trigger this module to print status information, over serial.

debugPrint_* functions are used by the openserial module to continuously print
status information about several modules in the OpenWSN stack.

\returns TRUE if this function printed something, FALSE otherwise.
*/
bool debugPrint_myDAGrank() {
   uint16_t output[2];
   
   memset(output,0,sizeof(uint16_t)*2);
   
   output[0] = neighbors_getMyDAGrank();
   output[1] = neighbors_getMyLowestDAGrank();
   openserial_printStatus(STATUS_DAGRANK,(uint8_t*)output,sizeof(uint16_t)*2);
   return TRUE;
}

/**
\brief Trigger this module to print status information, over serial.

debugPrint_* functions are used by the openserial module to continuously print
status information about several modules in the OpenWSN stack.

\returns TRUE if this function printed something, FALSE otherwise.
*/
bool debugPrint_kaPeriod() {
   uint16_t output;
   
   output = sixtop_vars.kaPeriod;
   
   openserial_printStatus(
       STATUS_KAPERIOD,
       (uint8_t*)&output,
       sizeof(output)
   );
   return TRUE;
}

//=========================== private =========================================

/**
\brief Transfer packet to MAC.

This function adds a IEEE802.15.4 header to the packet and leaves it the 
OpenQueue buffer. The very last thing it does is assigning this packet to the 
virtual component COMPONENT_SIXTOP_TO_IEEE802154E. Whenever it gets a change,
IEEE802154E will handle the packet.

\param[in] msg The packet to the transmitted
\param[in] iePresent Indicates wheter an Information Element is present in the
   packet.
\param[in] frameVersion The frame version to write in the packet.

\returns E_SUCCESS iff successful.
*/
owerror_t sixtop_send_internal(
   OpenQueueEntry_t* msg, 
   bool    payloadIEPresent) {
   
   // if there's a dedicated cell to the next hop parent, mark the packet for
   // allowing destination MAC address to be changed on the fly
   if (
         (neighbors_isParent(&(msg->l2_nextORpreviousHop))==TRUE)
         &&
         (schedule_getNumTxCellsToNeighbor(&(msg->l2_nextORpreviousHop)) > 0)
         &&
         (msg->l3_destinationAdd.type != ADDR_NONE)
         &&
         (packetfunctions_isBroadcastMulticast(&(msg->l3_destinationAdd))==FALSE)
      ) {
      msg->l2_couldChangeNextHop = TRUE;
   }
   
   // assign a number of retries
   if (
      packetfunctions_isBroadcastMulticast(&(msg->l2_nextORpreviousHop))==TRUE
      ) {
      msg->l2_retriesLeft = 1;
   } else {
      msg->l2_retriesLeft = TXRETRIES + 1;
      neighbors_updateEnqueuedPacketsToNeighbor(&(msg->l2_nextORpreviousHop));
   }
   // record this packet's dsn (for matching the ACK)
   msg->l2_dsn = sixtop_vars.dsn++;
   // this is a new packet which I never attempted to send
   msg->l2_numTxAttempts = 0;
   // transmit with the default TX power
   msg->l1_txPower = TX_POWER;
   // add a IEEE802.15.4 header
   ieee802154_prependHeader(msg,
                            msg->l2_frameType,
                            payloadIEPresent,
                            msg->l2_dsn,
                            &(msg->l2_nextORpreviousHop)
                            );
   // change owner to IEEE802154E fetches it from queue
   msg->owner  = COMPONENT_SIXTOP_TO_IEEE802154E;
   return E_SUCCESS;
}

// timer interrupt callbacks

void sixtop_maintenance_timer_cb(opentimer_id_t id) {
   scheduler_push_task(timer_sixtop_management_fired,TASKPRIO_SIXTOP);
}

void sixtop_timeout_timer_cb(opentimer_id_t id) {
   scheduler_push_task(timer_sixtop_six2six_timeout_fired,TASKPRIO_SIXTOP_TIMEOUT);
}

//======= EB/KA/maintainance task

/**
\brief Timer handlers which triggers MAC management task.

This function is called in task context by the scheduler after the RES timer
has fired. This timer is set to fire every second, on average.

The body of this function executes one of the MAC management task.
*/
void timer_sixtop_management_fired(void) {
   
   sixtop_vars.mgtTaskCounter = (sixtop_vars.mgtTaskCounter+1)%sixtop_vars.ebPeriod;
   
   switch (sixtop_vars.mgtTaskCounter) {
      case 0:
         // called every EBPERIOD seconds
         sixtop_sendEB();
         break;
      case 1:
         // called every EBPERIOD seconds
         neighbors_removeOld();
         break;
      case 2:
         // called every EBPERIOD seconds
         sixtop_maintaining();
         break;
      default:
         // called every second, except three times every EBPERIOD seconds
         sixtop_sendKA();
         break;
   }
}

/**
\brief Send an EB.

This is one of the MAC management tasks. This function inlines in the
timers_res_fired() function, but is declared as a separate function for better
readability of the code.
*/
port_INLINE void sixtop_sendEB() {
   OpenQueueEntry_t* eb;
   uint8_t len;
   
   len = 0;
   
   if ((ieee154e_isSynch()==FALSE) || (neighbors_getMyDAGrank()==DEFAULTDAGRANK)){
      // I'm not sync'ed or I did not acquire a DAGrank
      
      // delete packets genereted by this module (EB and KA) from openqueue
      openqueue_removeAllCreatedBy(COMPONENT_SIXTOP);
      
      // I'm now busy sending an EB
      sixtop_vars.busySendingEB = FALSE;
      
      // stop here
      return;
   }
   
   if (sixtop_vars.busySendingEB==TRUE) {
      // don't continue if I'm still sending a previous EB
      return;
   }
   
   // if I get here, I will send an EB
   
   // get a free packet buffer
   eb = openqueue_getFreePacketBuffer(COMPONENT_SIXTOP);
   if (eb==NULL) {
      openserial_printError(COMPONENT_SIXTOP,ERR_NO_FREE_PACKET_BUFFER,
                            (errorparameter_t)0,
                            (errorparameter_t)0);
      return;
   }
   
   // declare ownership over that packet
   eb->creator = COMPONENT_SIXTOP;
   eb->owner   = COMPONENT_SIXTOP;
   
   // reserve space for EB-specific header
   // reserving for IEs.
   len += processIE_prependSlotframeLinkIE(eb);
   len += processIE_prependChannelHoppingIE(eb);
   len += processIE_prependTSCHTimeslotIE(eb);
   len += processIE_prependSyncIE(eb);
   
   //add IE header 
   processIE_prependMLMEIE(eb,len);
  
   // some l2 information about this packet
   eb->l2_frameType                     = IEEE154_TYPE_BEACON;
   eb->l2_nextORpreviousHop.type        = ADDR_16B;
   eb->l2_nextORpreviousHop.addr_16b[0] = 0xff;
   eb->l2_nextORpreviousHop.addr_16b[1] = 0xff;
   
   //I has an IE in my payload
   eb->l2_payloadIEpresent = TRUE;

   // set l2-security attributes
   eb->l2_securityLevel   = IEEE802154_SECURITY_LEVEL_BEACON;
   eb->l2_keyIdMode       = IEEE802154_SECURITY_KEYIDMODE;
   eb->l2_keyIndex        = IEEE802154_SECURITY_K1_KEY_INDEX;

   // put in queue for MAC to handle
   sixtop_send_internal(eb,eb->l2_payloadIEpresent);
   
   // I'm now busy sending an EB
   sixtop_vars.busySendingEB = TRUE;
}

/**
\brief Send an keep-alive message, if necessary.

This is one of the MAC management tasks. This function inlines in the
timers_res_fired() function, but is declared as a separate function for better
readability of the code.
*/
port_INLINE void sixtop_sendKA() {
   OpenQueueEntry_t* kaPkt;
   open_addr_t*      kaNeighAddr;
   
   if (ieee154e_isSynch()==FALSE) {
      // I'm not sync'ed
      
      // delete packets genereted by this module (EB and KA) from openqueue
      openqueue_removeAllCreatedBy(COMPONENT_SIXTOP);
      
      sixtop_vars.busySendingEB = FALSE;
      
      // stop here
      return;
   }
   
   kaNeighAddr = neighbors_getKANeighbor(sixtop_vars.kaPeriod);
   if (kaNeighAddr==NULL) {
      // don't proceed if I have no neighbor I need to send a KA to
      return;
   }
   
   // if I get here, I will send a KA
   
   // get a free packet buffer
   kaPkt = openqueue_getFreePacketBuffer(COMPONENT_SIXTOP);
   if (kaPkt==NULL) {
      openserial_printError(COMPONENT_SIXTOP,ERR_NO_FREE_PACKET_BUFFER,
                            (errorparameter_t)1,
                            (errorparameter_t)0);
      return;
   }
   
   // declare ownership over that packet
   kaPkt->creator = COMPONENT_SIXTOP;
   kaPkt->owner   = COMPONENT_SIXTOP;
   
   // some l2 information about this packet
   kaPkt->l2_frameType = IEEE154_TYPE_DATA;
   memcpy(&(kaPkt->l2_nextORpreviousHop),kaNeighAddr,sizeof(open_addr_t));
   
   // set l2-security attributes
   kaPkt->l2_securityLevel   = IEEE802154_SECURITY_LEVEL;
   kaPkt->l2_keyIdMode       = IEEE802154_SECURITY_KEYIDMODE;
   kaPkt->l2_keyIndex        = IEEE802154_SECURITY_K2_KEY_INDEX;

   // put in queue for MAC to handle
   sixtop_send_internal(kaPkt,FALSE);
   
#ifdef OPENSIM
   debugpins_ka_set();
   debugpins_ka_clr();
#endif
}

void sixtop_maintaining(){
   open_addr_t neighbor;
   
   if (neighbors_getMyLowestDAGrank()==DEFAULTDAGRANK) {
      return;
   }
   
   // filter parameters
   if (sixtop_vars.six2six_state!=SIX_IDLE){
      return;
   }
   
   neighbor.type = ADDR_NONE;
   
   sixtop_removeCells_private(&neighbor,1,SIX_HANDLER_MAINTAIN);
}

//======= six2six task

void timer_sixtop_six2six_timeout_fired(void) {
   if (sixtop_vars.six2six_state == SIX_WAIT_ADDACK) {
      sixtop_removeCellsByState(&(sixtop_vars.schedule_ie),&(sixtop_vars.currentNeighbor));
   }
   openqueue_sixtopSetStopSendingOnShared(FALSE);
   sixtop_resetState(E_FAIL);
}

void sixtop_six2six_sendDone(OpenQueueEntry_t* msg, owerror_t error){
   six2six_handler_t originalHandler;
   open_addr_t       addressAddCells;
   
   addressAddCells.type = ADDR_NONE;
   originalHandler = sixtop_vars.handler;
   msg->owner = COMPONENT_SIXTOP_RES;
   
   switch (sixtop_vars.six2six_state) {
      case SIX_WAIT_ADDREQUEST_SENDDONE:
         if (error == E_SUCCESS) {
            // arm timeout
            opentimers_setPeriod(
               sixtop_vars.timeoutTimerId,
               TIME_MS,
               SIX2SIX_TIMEOUT_MS
            );
            opentimers_restart(sixtop_vars.timeoutTimerId);
            openqueue_sixtopSetStopSendingOnShared(TRUE);
            sixtop_vars.six2six_state = SIX_WAIT_ADDRESPONSE;
         } else {
            sixtop_resetState(E_FAIL);
         }
         break;
      case SIX_WAIT_ADDRESPONSE_SENDDONE:
         // arm timeout
         opentimers_setPeriod(
            sixtop_vars.timeoutTimerId,
            TIME_MS,
            SIX2SIX_TIMEOUT_MS
         );
         opentimers_restart(sixtop_vars.timeoutTimerId);
         openqueue_sixtopSetStopSendingOnShared(TRUE);
         sixtop_vars.six2six_state = SIX_WAIT_ADDACK;
         break;
      case SIX_WAIT_ADDACK_SENDDONE:
         sixtop_resetState(E_SUCCESS);
         break;
      case SIX_WAIT_REMOVEREQUEST_SENDDONE:
         if (error == E_SUCCESS) {
            sixtop_removeCellsByState(&(sixtop_vars.schedule_ie),&(sixtop_vars.currentNeighbor));
         }
         sixtop_resetState(error);
         if ((error == E_SUCCESS) && (originalHandler == SIX_HANDLER_MAINTAIN)) {
            memcpy(&addressAddCells,&(msg->l2_nextORpreviousHop),sizeof(open_addr_t));
            sixtop_addCells(&(msg->l2_nextORpreviousHop),1,SIX_HANDLER_MAINTAIN);
         }
         break;
      default:
         //log error
         break;
   }
   
   // discard reservation packets this component has created
   openqueue_freePacketBuffer(msg);
   if (addressAddCells.type!=ADDR_NONE) {
      sixtop_addCells(&addressAddCells,1,SIX_HANDLER_MAINTAIN);
   }
}

port_INLINE bool sixtop_processIEs(OpenQueueEntry_t* pkt, uint16_t * lenIE) {
   uint8_t ptr;
   uint8_t temp_8b,gr_elem_id,subid;
   uint16_t temp_16b,len,sublen;
   opcode_IE_ht opcode_ie;
   bandwidth_IE_ht bandwidth_ie;
   schedule_IE_ht schedule_ie;
 
   ptr=0; 
   memset(&opcode_ie,0,sizeof(opcode_IE_ht));
   memset(&bandwidth_ie,0,sizeof(bandwidth_IE_ht));
   memset(&schedule_ie,0,sizeof(schedule_IE_ht));  
  
   //candidate IE header  if type ==0 header IE if type==1 payload IE
   temp_8b = *((uint8_t*)(pkt->payload)+ptr);
   ptr++;
   temp_16b = temp_8b + ((*((uint8_t*)(pkt->payload)+ptr))<<8);
   ptr++;
   *lenIE = ptr;
   if(
      (temp_16b & IEEE802154E_DESC_TYPE_PAYLOAD_IE) == 
      IEEE802154E_DESC_TYPE_PAYLOAD_IE
   ){
   //payload IE - last bit is 1
      len = temp_16b & IEEE802154E_DESC_LEN_PAYLOAD_IE_MASK;
      gr_elem_id = 
         (temp_16b & IEEE802154E_DESC_GROUPID_PAYLOAD_IE_MASK)>>
         IEEE802154E_DESC_GROUPID_PAYLOAD_IE_SHIFT;
   }else {
   //header IE - last bit is 0
      len = temp_16b & IEEE802154E_DESC_LEN_HEADER_IE_MASK;
      gr_elem_id = (temp_16b & IEEE802154E_DESC_ELEMENTID_HEADER_IE_MASK)>>
         IEEE802154E_DESC_ELEMENTID_HEADER_IE_SHIFT; 
   }
  
   *lenIE += len;
   //now determine sub elements if any
   switch(gr_elem_id){
      //this is the only groupID that we parse. See page 82.  
      case IEEE802154E_MLME_IE_GROUPID:
        //IE content can be any of the sub-IEs. Parse and see which
        do{
           //read sub IE header
           temp_8b = *((uint8_t*)(pkt->payload)+ptr);
           ptr = ptr + 1;
           temp_16b = temp_8b + ((*((uint8_t*)(pkt->payload)+ptr))<<8);
           ptr = ptr + 1;
           len = len - 2; //remove header fields len
           if(
              (temp_16b & IEEE802154E_DESC_TYPE_LONG) == 
              IEEE802154E_DESC_TYPE_LONG
              ){
              //long sub-IE - last bit is 1
              sublen = temp_16b & IEEE802154E_DESC_LEN_LONG_MLME_IE_MASK;
              subid= 
                 (temp_16b & IEEE802154E_DESC_SUBID_LONG_MLME_IE_MASK)>>
                 IEEE802154E_DESC_SUBID_LONG_MLME_IE_SHIFT; 
           } else {
              //short IE - last bit is 0
              sublen = temp_16b & IEEE802154E_DESC_LEN_SHORT_MLME_IE_MASK;
              subid = (temp_16b & IEEE802154E_DESC_SUBID_SHORT_MLME_IE_MASK)>>
                 IEEE802154E_DESC_SUBID_SHORT_MLME_IE_SHIFT; 
           }
           switch(subid){
              case MLME_IE_SUBID_OPCODE:
              processIE_retrieveOpcodeIE(pkt,&ptr,&opcode_ie);
              break;
              case MLME_IE_SUBID_BANDWIDTH:
              processIE_retrieveBandwidthIE(pkt,&ptr,&bandwidth_ie);
              break;
              case MLME_IE_SUBID_TRACKID:
              break;
              case MLME_IE_SUBID_SCHEDULE:
              processIE_retrieveScheduleIE(pkt,&ptr,&schedule_ie);
              break;
          default:
             return FALSE;
             break;
        }
        len = len - sublen;
      } while(len>0);
      
      break;
    default:
      *lenIE = 0;//no header or not recognized.
       return FALSE;
   }
   if (*lenIE>127) {
         // log the error
     openserial_printError(COMPONENT_IEEE802154E,ERR_HEADER_TOO_LONG,
                           (errorparameter_t)*lenIE,
                           (errorparameter_t)1);
   }
  
   if(*lenIE>0) {
      sixtop_notifyReceiveCommand(&opcode_ie,
                                  &bandwidth_ie,
                                  &schedule_ie,
                                  &(pkt->l2_nextORpreviousHop));
   }
  
  return TRUE;
}

void sixtop_notifyReceiveCommand(
      opcode_IE_ht* opcode_ie, 
      bandwidth_IE_ht* bandwidth_ie, 
      schedule_IE_ht* schedule_ie,
      open_addr_t* addr
   ) {
   
   switch(opcode_ie->opcode) {
      case SIXTOP_SOFT_CELL_REQUEST:
         //received uResCommand is reserve link request
         if (sixtop_vars.six2six_state == SIX_IDLE) {
            memcpy(&(sixtop_vars.opcode_ie),opcode_ie,sizeof(opcode_IE_ht));
            sixtop_vars.opcode_ie.opcode = SIXTOP_SOFT_CELL_RESPONSE;
            memcpy(&(sixtop_vars.bandwidth_ie),bandwidth_ie,sizeof(bandwidth_IE_ht));
            memcpy(&(sixtop_vars.schedule_ie),schedule_ie,sizeof(schedule_IE_ht));
            // need to check whether the links are available to be scheduled.
            if (
                  (sixtop_vars.bandwidth_ie.numOfLinks > SCHEDULEIEMAXNUMCELLS)
                  ||
                  (sixtop_vars.schedule_ie.numberOfcells > SCHEDULEIEMAXNUMCELLS)
                  ||
                  (sixtop_vars.bandwidth_ie.numOfLinks > sixtop_vars.schedule_ie.numberOfcells)
                  ||
                  (sixtop_vars.bandwidth_ie.slotframeID != sixtop_vars.schedule_ie.frameID)
                  ||
                  (sixtop_areAvailableCellsToBeScheduled() == FALSE)
               ) {
               sixtop_vars.bandwidth_ie.numOfLinks = 0;
            }
            memcpy(&(sixtop_vars.currentNeighbor),addr,sizeof(open_addr_t));
            
            if ((sixtop_sendCommand() == E_SUCCESS) && (sixtop_vars.bandwidth_ie.numOfLinks > 0)) {
               sixtop_vars.six2six_state = SIX_WAIT_ADDRESPONSE_SENDDONE;
               sixtop_addCellsByState();
            } else {
               sixtop_resetState(E_FAIL);
            }
         }
         break;
      case SIXTOP_SOFT_CELL_RESPONSE:
         //received uResCommand is reserve link response
         if (
               ((sixtop_vars.six2six_state == SIX_WAIT_ADDREQUEST_SENDDONE)||(sixtop_vars.six2six_state == SIX_WAIT_ADDRESPONSE))
               &&
               (packetfunctions_sameAddress(addr,&(sixtop_vars.currentNeighbor)) == TRUE)
               &&
               (opcode_ie->seqNumber == sixtop_vars.opcode_ie.seqNumber)
            ) {
            opentimers_stop(sixtop_vars.timeoutTimerId);
            openqueue_sixtopSetStopSendingOnShared(FALSE);
            openqueue_removeAllCreatedBy(COMPONENT_SIXTOP_RES);
            sixtop_vars.opcode_ie.opcode = SIXTOP_SOFT_CELL_ACK;
            memcpy(&(sixtop_vars.bandwidth_ie),bandwidth_ie,sizeof(bandwidth_IE_ht));
            memcpy(&(sixtop_vars.schedule_ie),schedule_ie,sizeof(schedule_IE_ht));
            if (
                  (sixtop_vars.bandwidth_ie.numOfLinks != 0)
                  &&
                  (sixtop_vars.bandwidth_ie.numOfLinks <= SCHEDULEIEMAXNUMCELLS)
                  &&
                  (sixtop_vars.bandwidth_ie.numOfLinks == sixtop_vars.schedule_ie.numberOfcells)
                  &&
                  (sixtop_vars.bandwidth_ie.slotframeID == sixtop_vars.schedule_ie.frameID)
                  &&
                  (sixtop_areAvailableCellsToBeScheduled() == TRUE)
               ) {
               if (sixtop_sendCommand() == E_SUCCESS) {
                  sixtop_addCellsByState();
                  sixtop_vars.six2six_state = SIX_WAIT_ADDACK_SENDDONE;
               } else {
                  sixtop_resetState(E_FAIL);
               }
            } else {
               sixtop_resetState(E_FAIL);
            }
         }
         break;
      case SIXTOP_SOFT_CELL_REMOVE:
         //received uResComand is remove link request
         sixtop_removeCellsByState(schedule_ie,addr);
         break;
      case SIXTOP_SOFT_CELL_ACK:
         //received uResCommand is reserve link ack
         if (
               ((sixtop_vars.six2six_state == SIX_WAIT_ADDRESPONSE_SENDDONE)||(sixtop_vars.six2six_state == SIX_WAIT_ADDACK))
               &&
               (packetfunctions_sameAddress(addr,&(sixtop_vars.currentNeighbor))==TRUE)
               &&
               (opcode_ie->seqNumber == sixtop_vars.opcode_ie.seqNumber)
            ) {
            opentimers_stop(sixtop_vars.timeoutTimerId);
            openqueue_sixtopSetStopSendingOnShared(FALSE);
            openqueue_removeAllCreatedBy(COMPONENT_SIXTOP_RES);
            sixtop_resetState(E_SUCCESS);
         }
         break;
      default:
         // log the error
         break;
   }
}

owerror_t sixtop_sendCommand() {
   OpenQueueEntry_t* sixtopPkt;
   uint8_t len;
   
   // get a free packet buffer
   sixtopPkt = openqueue_getSureFreePacketBuffer(COMPONENT_SIXTOP_RES);
  
   if(sixtopPkt==NULL) {
      openserial_printError(
         COMPONENT_SIXTOP_RES,
         ERR_NO_FREE_PACKET_BUFFER,
         (errorparameter_t)0,
         (errorparameter_t)0);
      return E_FAIL;
   }
   
   // declare ownership over that packet
   sixtopPkt->creator = COMPONENT_SIXTOP_RES;
   sixtopPkt->owner   = COMPONENT_SIXTOP_RES;
    
   memcpy(&(sixtopPkt->l2_nextORpreviousHop),&(sixtop_vars.currentNeighbor),sizeof(open_addr_t));
   
   len = 0;
   
   //add ScheduleIE
   len += processIE_prependScheduleIE(
      sixtopPkt,
      &(sixtop_vars.schedule_ie));
   
   if (sixtop_vars.opcode_ie.opcode != SIXTOP_SOFT_CELL_REMOVE) {
      //add BandwidthIE
      len += processIE_prependBandwidthIE(
         sixtopPkt,
         &(sixtop_vars.bandwidth_ie));
   }
   
   //add opcodeIE
   len += processIE_prependOpcodeIE(
      sixtopPkt,
      &(sixtop_vars.opcode_ie));
   
   //add IE header 
   processIE_prependMLMEIE(
      sixtopPkt,
      len);
    
   //I have an IE in my payload
   sixtopPkt->l2_payloadIEpresent = TRUE;
   
   return sixtop_send(sixtopPkt);
}

//======= helper functions

owerror_t sixtop_removeCells_private(open_addr_t* neighbor, uint16_t numCells, six2six_handler_t handler){
   bool              outcome;
   
   // build opcode_ie
   sixtop_vars.opcode_ie.opcode = SIXTOP_SOFT_CELL_REMOVE;
   sixtop_vars.opcode_ie.seqNumber = sixtop_vars.seqNumber;
   
   // start building schedule_ie
   sixtop_vars.schedule_ie.frameID = schedule_getFrameHandle();
   sixtop_vars.schedule_ie.numberOfcells = numCells;
   
   // generate candidate cell list and continue building schedule_ie
   outcome = sixtop_candidateRemoveCellList(neighbor);
   if (outcome == FALSE) {
      sixtop_resetState(E_FAIL);
      return E_FAIL;
   }
   
   // store neighbor
   memcpy(&(sixtop_vars.currentNeighbor),neighbor,sizeof(open_addr_t));
   
   if (sixtop_sendCommand() == E_FAIL) {
      sixtop_resetState(E_FAIL);
      return E_FAIL;
   }
   
   // update state
   sixtop_vars.six2six_state = SIX_WAIT_REMOVEREQUEST_SENDDONE;
   sixtop_vars.handler = handler;
   return E_SUCCESS;
}

bool sixtop_candidateAddCellList() {
   uint16_t counter;
   uint16_t numCandCells;
   uint16_t candCells[SCHEDULEIEMAXNUMCELLS];
   
   if (sixtop_vars.bandwidth_ie.numOfLinks > SCHEDULEIEMAXNUMCELLS) {
      return FALSE;
   }
   
   memset(&candCells,0,sizeof(uint16_t)*SCHEDULEIEMAXNUMCELLS);
   numCandCells = SCHEDULEIEMAXNUMCELLS;
   
   schedule_getRandomAvailableSlotOffsets(&numCandCells, &candCells[0]);
   
   if (numCandCells < sixtop_vars.bandwidth_ie.numOfLinks) {
      return FALSE;
   }
   
   // build schedule_ie
   sixtop_vars.schedule_ie.type = 1;
   sixtop_vars.schedule_ie.frameID = sixtop_vars.bandwidth_ie.slotframeID;
   sixtop_vars.schedule_ie.flag = 1; // the cells listed in cellList are available to be schedule.
   for (counter=0;counter<SCHEDULEIEMAXNUMCELLS;counter++) {
      if (counter < numCandCells) {
         sixtop_vars.schedule_ie.cellList[counter].tsNum       = candCells[counter];
         sixtop_vars.schedule_ie.cellList[counter].choffset    = openrandom_get16b() % 15 + 1;
         sixtop_vars.schedule_ie.cellList[counter].linkoptions = CELLTYPE_TX;
      } else {
         sixtop_vars.schedule_ie.cellList[counter].linkoptions = CELLTYPE_OFF;
      }
   }
   
   return TRUE;
}

bool sixtop_candidateRemoveCellList(
      open_addr_t* neighbor
   ) {
   uint8_t              counter;
   scheduleEntry_t*     entry;
   bool                 maintaining;
   
   maintaining = FALSE;
   if (neighbor->type == ADDR_NONE) {
      maintaining = TRUE;
   }
   
   if (sixtop_vars.schedule_ie.numberOfcells > SCHEDULEIEMAXNUMCELLS) {
      sixtop_vars.schedule_ie.numberOfcells = SCHEDULEIEMAXNUMCELLS;
   }
   
   // build schedule_ie
   sixtop_vars.schedule_ie.type = 1;
   sixtop_vars.schedule_ie.flag = 1;
   
   entry = NULL;
   for (counter=0;counter<sixtop_vars.schedule_ie.numberOfcells;counter++) {
      entry = schedule_statistic_poorLinkQuality(neighbor,entry,maintaining);
      if (entry != NULL) {
         sixtop_vars.schedule_ie.cellList[counter].tsNum       = entry->slotOffset;
         sixtop_vars.schedule_ie.cellList[counter].choffset    = entry->channelOffset;
         sixtop_vars.schedule_ie.cellList[counter].linkoptions = CELLTYPE_TX;
      } else {
         break;
      }
   }
   sixtop_vars.schedule_ie.numberOfcells = counter;
   for (counter=sixtop_vars.schedule_ie.numberOfcells;counter<SCHEDULEIEMAXNUMCELLS;counter++) {
      sixtop_vars.schedule_ie.cellList[counter].linkoptions = CELLTYPE_OFF;
   }
   
   if (sixtop_vars.schedule_ie.numberOfcells == 0) {
      return FALSE;
   }
   
   return TRUE;
}

void sixtop_addCellsByState() {
   uint8_t     counter;
  
   //set schedule according links
   for(counter=0;counter<SCHEDULEIEMAXNUMCELLS;counter++){
      //only schedule when the request side wants to schedule a tx cell
      if(sixtop_vars.schedule_ie.cellList[counter].linkoptions == CELLTYPE_TX){
         switch(sixtop_vars.six2six_state) {
            case SIX_WAIT_ADDRESPONSE_SENDDONE:
               //add a RX link
               schedule_addActiveSlot(
                  sixtop_vars.schedule_ie.cellList[counter].tsNum,
                  CELLTYPE_RX,
                  FALSE,
                  sixtop_vars.schedule_ie.cellList[counter].choffset,
                  &(sixtop_vars.currentNeighbor));
               break;
            case SIX_WAIT_ADDREQUEST_SENDDONE:
            case SIX_WAIT_ADDRESPONSE:
               //add a TX link
               schedule_addActiveSlot(
                  sixtop_vars.schedule_ie.cellList[counter].tsNum,
                  CELLTYPE_TX,
                  FALSE,
                  sixtop_vars.schedule_ie.cellList[counter].choffset,
                  &(sixtop_vars.currentNeighbor));
               break;
            default:
               //log error
               break;
         }
      }
   }
}

void sixtop_removeCellsByState(
      schedule_IE_ht* schedule_ie,
      open_addr_t* neighbor
   ) {
   uint8_t counter;
   
   for(counter=0;counter<schedule_ie->numberOfcells;counter++) {
      if ((schedule_ie->cellList[counter].linkoptions == CELLTYPE_TX) || (schedule_ie->cellList[counter].linkoptions == CELLTYPE_RX)) {
         if (schedule_isSlotOffsetAvailable(schedule_ie->cellList[counter].tsNum) == FALSE) {
            schedule_removeActiveSlot(schedule_ie->cellList[counter].tsNum,neighbor);
         }
      }
   }
}

bool sixtop_areAvailableCellsToBeScheduled() {
   uint8_t counter;
   uint8_t bw;
   bool    available;
   
   counter    = 0;
   bw         = sixtop_vars.bandwidth_ie.numOfLinks;
   available  = FALSE;
   
   for (counter=0;counter<SCHEDULEIEMAXNUMCELLS;counter++) {
      if ((bw>0) && (schedule_isSlotOffsetAvailable(sixtop_vars.schedule_ie.cellList[counter].tsNum) == TRUE)) {
         bw--;
      } else {
         sixtop_vars.schedule_ie.cellList[counter].linkoptions = CELLTYPE_OFF;
      }
   }
   available = FALSE;
   if (bw==0) {
      // local schedule can satisfy the bandwidth of cell request. 
      available = TRUE;
   }
   
   return available;
}

void sixtop_resetState(owerror_t error) {
   uint16_t counter;
   memset(&(sixtop_vars.opcode_ie),0,sizeof(opcode_IE_ht));
   memset(&(sixtop_vars.bandwidth_ie),0,sizeof(bandwidth_IE_ht));
   memset(&(sixtop_vars.schedule_ie),0,sizeof(schedule_IE_ht));
   for (counter=0;counter<SCHEDULEIEMAXNUMCELLS;counter++) {
      sixtop_vars.schedule_ie.cellList[counter].linkoptions = CELLTYPE_OFF;
   }
   memset(&(sixtop_vars.currentNeighbor),0,sizeof(open_addr_t));
   if (
         (sixtop_vars.six2six_state == SIX_WAIT_ADDREQUEST_SENDDONE)
         ||
         (sixtop_vars.six2six_state == SIX_WAIT_ADDRESPONSE)
         ||
         (sixtop_vars.six2six_state == SIX_WAIT_ADDACK_SENDDONE)
         ||
         (sixtop_vars.six2six_state == SIX_WAIT_REMOVEREQUEST_SENDDONE)
      ) {
      sixtop_vars.seqNumber++;
   }
   if (sixtop_vars.handler == SIX_HANDLER_OTF) {
      otf_notif_release(error);
   }
   sixtop_vars.handler = SIX_HANDLER_NONE;
   sixtop_vars.six2six_state = SIX_IDLE;
}
