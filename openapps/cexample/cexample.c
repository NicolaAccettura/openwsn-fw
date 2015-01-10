/**
\brief An example CoAP application.
*/

#include "opendefs.h"
#include "cexample.h"
#include "opencoap.h"
#include "opentimers.h"
#include "openqueue.h"
#include "packetfunctions.h"
#include "openserial.h"
#include "openrandom.h"
#include "scheduler.h"
#include "idmanager.h"
#include "IEEE802154E.h"
#include "idmanager.h"

//=========================== defines =========================================

/// inter-packet period (in ms)
#define CEXAMPLEPERIOD  5000
#define PAYLOADLEN      62

const uint8_t cexample_path0[] = "ex";
uint8_t indexPacket = 0;

//=========================== variables =======================================

cexample_vars_t cexample_vars;

//=========================== prototypes ======================================

owerror_t cexample_receive(OpenQueueEntry_t* msg,
                    coap_header_iht*  coap_header,
                    coap_option_iht*  coap_options);
void    cexample_timer_cb(void);
void    cexample_task_cb(void);
void    cexample_sendDone(OpenQueueEntry_t* msg,
                       owerror_t error);

//=========================== public ==========================================

void cexample_init() {
   
   // prepare the resource descriptor for the /ex path
   cexample_vars.desc.path0len             = sizeof(cexample_path0)-1;
   cexample_vars.desc.path0val             = (uint8_t*)(&cexample_path0);
   cexample_vars.desc.path1len             = 0;
   cexample_vars.desc.path1val             = NULL;
   cexample_vars.desc.componentID          = COMPONENT_CEXAMPLE;
   cexample_vars.desc.callbackRx           = &cexample_receive;
   cexample_vars.desc.callbackSendDone     = &cexample_sendDone;
   
   
   opencoap_register(&cexample_vars.desc);
   cexample_vars.timerId    = opentimers_start(CEXAMPLEPERIOD,
                                                TIMER_PERIODIC,TIME_MS,
                                                cexample_timer_cb);
}

//=========================== private =========================================

owerror_t cexample_receive(OpenQueueEntry_t* msg,
                      coap_header_iht* coap_header,
                      coap_option_iht* coap_options) {
   return E_FAIL;
}

//timer fired, but we don't want to execute task in ISR mode
//instead, push task to scheduler with COAP priority, and let scheduler take care of it
void cexample_timer_cb(){
   scheduler_push_task(cexample_task_cb,TASKPRIO_COAP);
}

void cexample_task_cb() {
   OpenQueueEntry_t*    pkt;
   owerror_t            outcome;
   uint8_t              numOptions;
   
   // don't run if not synch
   if (ieee154e_isSynch() == FALSE) return;
   
   // don't run on dagroot
   if (idmanager_getIsDAGroot()) {
      opentimers_stop(cexample_vars.timerId);
      return;
   }
   
   // create a CoAP RD packet
   pkt = openqueue_getFreePacketBuffer(COMPONENT_CEXAMPLE);
   if (pkt==NULL) {
      openserial_printError(
         COMPONENT_CEXAMPLE,
         ERR_NO_FREE_PACKET_BUFFER,
         (errorparameter_t)0,
         (errorparameter_t)0
      );
      openqueue_freePacketBuffer(pkt);
      return;
   }
   // take ownership over that packet
   pkt->creator                   = COMPONENT_CEXAMPLE;
   pkt->owner                     = COMPONENT_CEXAMPLE;
   // CoAP payload
   packetfunctions_reserveHeaderSize(pkt,4);
   pkt->payload[0]                = 0xff;
   pkt->payload[1]                = idmanager_getMyID(ADDR_64B)->addr_64b[6];
   pkt->payload[2]                = idmanager_getMyID(ADDR_64B)->addr_64b[7];
   pkt->payload[3]                = indexPacket;
   indexPacket++;
   
   numOptions = 0;
   // location-path option
   packetfunctions_reserveHeaderSize(pkt,sizeof(cexample_path0)-1);
   memcpy(&pkt->payload[0],cexample_path0,sizeof(cexample_path0)-1);
   packetfunctions_reserveHeaderSize(pkt,1);
   pkt->payload[0]                = ((COAP_OPTION_NUM_URIPATH) << 4) | (sizeof(cexample_path0)-1);
   numOptions++;
   
   // metadata
   pkt->l4_destination_port       = WKP_UDP_COAP;
   pkt->l3_destinationAdd.type    = ADDR_128B;
   memcpy(&pkt->l3_destinationAdd.addr_128b[0],&ipAddr_ringmaster,16);
   
   // send
   outcome = opencoap_send(
      pkt,
      COAP_TYPE_NON,
      COAP_CODE_REQ_PUT,
      numOptions,
      &cexample_vars.desc
   );
   
   // avoid overflowing the queue if fails
   if (outcome==E_FAIL) {
      openqueue_freePacketBuffer(pkt);
   }

   return;
}

void cexample_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   openqueue_freePacketBuffer(msg);
}
