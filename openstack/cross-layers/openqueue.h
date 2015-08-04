#ifndef __OPENQUEUE_H
#define __OPENQUEUE_H

/**
\addtogroup cross-layers
\{
\addtogroup OpenQueue
\{
*/

#include "opendefs.h"
#include "IEEE802154.h"

//=========================== define ==========================================

#define QUEUELENGTH  10
#define NUMSUREFREEENTRIES 3

//=========================== typedef =========================================

typedef struct {
   uint8_t  creator;
   uint8_t  owner;
} debugOpenQueueEntry_t;

//=========================== module variables ================================

typedef struct {
   OpenQueueEntry_t queue[QUEUELENGTH];
   bool             stopSendingOnShared;
} openqueue_vars_t;

//=========================== prototypes ======================================

// admin
void               openqueue_init(void);
bool               debugPrint_queue(void);
// called by any component
OpenQueueEntry_t*  openqueue_getFreePacketBuffer(uint8_t creator);
OpenQueueEntry_t*  openqueue_getSureFreePacketBuffer(uint8_t creator);
owerror_t          openqueue_freePacketBuffer(OpenQueueEntry_t* pkt);
void               openqueue_removeAllCreatedBy(uint8_t creator);
void               openqueue_removeAllOwnedBy(uint8_t owner);
// called by res
OpenQueueEntry_t*  openqueue_sixtopGetSentPacket(void);
OpenQueueEntry_t*  openqueue_sixtopGetReceivedPacket(void);
uint8_t            openqueue_sixtopGetNumPacketsToNeighbor(open_addr_t* toNeighbor);
void               openqueue_sixtopSetStopSendingOnShared(bool stopSendingOnShared);
// called by IEEE80215E
OpenQueueEntry_t*  openqueue_macGetDataPacket(open_addr_t* toNeighbor, bool couldChangeNextHop, bool* willChangeNextHop);
OpenQueueEntry_t*  openqueue_macGetEBPacket(void);

/**
\}
\}
*/

#endif
