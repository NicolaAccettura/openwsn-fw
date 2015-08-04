#ifndef __SIXTOP_H
#define __SIXTOP_H

/**
\addtogroup MAChigh
\{
\addtogroup sixtop
\{
*/

#include "opentimers.h"
#include "opendefs.h"
#include "processIE.h"
//=========================== define ==========================================

enum sixtop_CommandID_num{
   SIXTOP_SOFT_CELL_REQUEST            = 0x00,
   SIXTOP_SOFT_CELL_RESPONSE           = 0x01,
   SIXTOP_SOFT_CELL_REMOVE             = 0x02,
   SIXTOP_SOFT_CELL_ACK                = 0x03
};

// states of the sixtop-to-sixtop state machine
typedef enum {
   SIX_IDLE                                     = 0x00,   // ready for next event
   // source
   SIX_WAIT_ADDREQUEST_SENDDONE                 = 0x01,   // waiting for SendDone confirmation
   SIX_WAIT_ADDRESPONSE                         = 0x02,   // waiting for response from the neighbor
   SIX_WAIT_ADDACK_SENDDONE                     = 0x03,   // waiting for SendDone confirmation
   SIX_WAIT_REMOVEREQUEST_SENDDONE              = 0x04,   // waiting for SendDone confirmation
   // destinations
   SIX_WAIT_ADDRESPONSE_SENDDONE                = 0x05,   // waiting for SendDone confirmation
   SIX_WAIT_ADDACK                              = 0x06    // waiting for ack from the neighbor
} six2six_state_t;

// before sixtop protocol is called, sixtop handler must be set
typedef enum {
   SIX_HANDLER_NONE                    = 0x00, // when complete reservation, handler must be set to none
   SIX_HANDLER_MAINTAIN                = 0x01, // the handler is maintenance process
   SIX_HANDLER_OTF                     = 0x02, // the handler is otf
   SIX_HANDLER_C6T                     = 0x03  // the handler is otf
} six2six_handler_t;

//=========================== typedef =========================================

#define SIX2SIX_TIMEOUT_MS 20000
#define SIXTOP_MINIMAL_EBPERIOD 5 // minist period of sending EB

//=========================== module variables ================================

typedef struct {
   uint16_t             periodMaintenance;
   bool                 busySendingKA;           // TRUE when busy sending a keep-alive
   bool                 busySendingEB;           // TRUE when busy sending an enhanced beacon
   uint8_t              dsn;                     // current data sequence number
   uint8_t              mgtTaskCounter;          // counter to determine what management task to do
   opentimer_id_t       maintenanceTimerId;
   opentimer_id_t       timeoutTimerId;          // TimeOut timer id
   uint16_t             kaPeriod;                // period of sending KA
   uint16_t             ebPeriod;                // period of sending EB
   six2six_state_t      six2six_state;
   six2six_handler_t    handler;
   uint8_t              seqNumber;
   opcode_IE_ht         opcode_ie;
   bandwidth_IE_ht      bandwidth_ie;
   schedule_IE_ht       schedule_ie;
   open_addr_t          currentNeighbor;
} sixtop_vars_t;

//=========================== prototypes ======================================

// admin
void      sixtop_init(void);
void      sixtop_setKaPeriod(uint16_t kaPeriod);
void      sixtop_setEBPeriod(uint8_t ebPeriod);
// scheduling
owerror_t sixtop_addCells(open_addr_t* neighbor, uint16_t numCells, six2six_handler_t handler);
owerror_t sixtop_removeCells(open_addr_t*  neighbor, uint16_t numCells, six2six_handler_t handler);
// from upper layer
owerror_t sixtop_send(OpenQueueEntry_t *msg);
// from lower layer
void      task_sixtopNotifSendDone(void);
void      task_sixtopNotifReceive(void);
void      task_sixtopNotifSlotframe(void);
// debugging
bool      debugPrint_myDAGrank(void);
bool      debugPrint_kaPeriod(void);

/**
\}
\}
*/

#endif
