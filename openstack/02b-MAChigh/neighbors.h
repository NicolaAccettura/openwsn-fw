#ifndef __NEIGHBORS_H
#define __NEIGHBORS_H

/**
\addtogroup MAChigh
\{
\addtogroup Neighbors
\{
*/
#include "opendefs.h"
#include "icmpv6rpl.h"
#include "opentimers.h"

//=========================== define ==========================================

#define MAXNUMNEIGHBORS           10
#define MAXPREFERENCE             2
#define INTERMEDIATEPREFERENCE    1
#define BADNEIGHBORMAXRSSI        -80 //dBm
#define GOODNEIGHBORMINRSSI       -90 //dBm
#define SWITCHSTABILITYTHRESHOLD  3
#define DEFAULTLINKCOST           16

#define DEFAULTJOINPRIORITY       255

#define MAXDAGRANK                0xffff
#define DEFAULTDAGRANK            MAXDAGRANK
#define MINHOPRANKINCREASE        256  //default value in RPL and Minimal 6TiSCH draft
#define PARENTSETSIZE             2
#define PARENTSWITCHTHRESHOLD     3 * MINHOPRANKINCREASE
#define MAXRANKINCREASE           4 * MINHOPRANKINCREASE

//=========================== typedef =========================================

typedef struct {
   bool             used;
   uint8_t          parentPreference;
   bool             stableNeighbor;
   uint8_t          switchStabilityCounter;
   open_addr_t      addr_64b;
   dagrank_t        DAGrank;
   dagrank_t        resultingDAGrank;
   int8_t           rssi;
   uint8_t          numRx;
   uint8_t          numTx;
   uint8_t          numTxACK;
   uint8_t          numWraps;//number of times the tx counter wraps. can be removed if memory is a restriction. also check openvisualizer then.
   asn_t            asn;
   uint8_t          joinPrio;
   uint8_t          requiredVirtualCells;
   uint8_t          aveRequiredVirtualCells;
} neighborRow_t;

BEGIN_PACK
typedef struct {
   uint8_t          row;
   bool             used;
   uint8_t          parentPreference;
   bool             stableNeighbor;
   uint8_t          switchStabilityCounter;
   open_addr_t      addr_64b;
   dagrank_t        DAGrank;
   int8_t           rssi;
   uint8_t          numRx;
   uint8_t          numTx;
   uint8_t          numTxACK;
   uint8_t          numWraps;//number of times the tx counter wraps. can be removed if memory is a restriction. also check openvisualizer then.
   asn_t            asn;
   uint8_t          joinPrio;
} debugNeighborEntry_t;
END_PACK

//=========================== module variables ================================
   
typedef struct {
   neighborRow_t        neighbors[MAXNUMNEIGHBORS];
   uint8_t              parents[PARENTSETSIZE];
   uint8_t              currentParentIndex;
   dagrank_t            myDAGrank;
   dagrank_t            myLowestDAGrank;
   uint8_t              debugRow;
   opentimer_id_t       timerId;
   uint32_t             cleanupDelay;
   icmpv6rpl_dio_ht*    dio; //keep it global to be able to debug correctly.
} neighbors_vars_t;

//=========================== prototypes ======================================

void          neighbors_init(void);

// getters
dagrank_t     neighbors_getMyDAGrank(void);
dagrank_t     neighbors_getMyLowestDAGrank(void);
uint8_t       neighbors_getNumNeighbors(void);
bool          neighbors_getPreferredParentEui64(open_addr_t* addressToWrite);
bool          neighbors_getParentByIndex(open_addr_t* addressToWrite, uint8_t parentIdx);
void          neighbors_getNextHopParent(open_addr_t* addressToWrite);
open_addr_t*  neighbors_getKANeighbor(uint16_t kaPeriod);
bool          neighbors_getOTFstatisticsByIndex(
   uint8_t        neighborIndex,
   open_addr_t*   address,
   uint8_t*       requiredVirtualCells,
   uint8_t*       aveRequiredVirtualCells
);
// setters
void          neighbors_setMyDAGrank(dagrank_t rank);
void          neighbors_setAdvertizedDAGrank(void);
void          neighbors_updateEnqueuedPacketsToNeighbor(open_addr_t* address);
void          neighbors_resetOTFstatistics(void);

// interrogators
bool          neighbors_isStableNeighbor(open_addr_t* address);
bool          neighbors_isPreferredParent(open_addr_t* address);
bool          neighbors_isParent(open_addr_t* address);
bool          neighbors_isNeighborWithLowerDAGrank(uint8_t index);
bool          neighbors_isNeighborWithHigherDAGrank(uint8_t index);

// updating neighbor information
void          neighbors_indicateRx(
   open_addr_t*         l2_src,
   int8_t               rssi,
   asn_t*               asnTimestamp,
   bool                 joinPrioPresent,
   uint8_t              joinPrio
);
void          neighbors_indicateTx(
   open_addr_t*         dest,
   bool                 succesfullTx,
   asn_t*               asnTimestamp
);
void          neighbors_indicateRxDIO(OpenQueueEntry_t* msg, bool* newDODAGID);

// get addresses
void          neighbors_getNeighbor(open_addr_t* address,uint8_t addr_type,uint8_t index);
// managing routing info
void          neighbors_updateMyDAGrankAndNeighborPreference(void);
// maintenance
void          neighbors_removeOld(void);
// debug
bool          debugPrint_neighbors(void);

/**
\}
\}
*/

#endif
