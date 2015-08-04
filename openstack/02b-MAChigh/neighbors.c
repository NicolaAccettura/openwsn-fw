#include "opendefs.h"
#include "neighbors.h"
#include "openqueue.h"
#include "packetfunctions.h"
#include "idmanager.h"
#include "openserial.h"
#include "IEEE802154E.h"
#include "opentimers.h"
#include "scheduler.h"
#include "leds.h"
#include "openrandom.h"

//=========================== variables =======================================

neighbors_vars_t neighbors_vars;

//=========================== prototypes ======================================

void registerNewNeighbor(
        open_addr_t* neighborID,
        int8_t       rssi,
        asn_t*       asnTimestamp,
        bool         joinPrioPresent,
        uint8_t      joinPrio
     );
uint8_t findNeighborRow(open_addr_t* neighbor);
void removeNeighbor(uint8_t neighborIndex);
uint8_t neighbors_getLowestJoinPriorityNeighborIndex(void);
bool isThisRowMatching(
        open_addr_t* address,
        uint8_t      rowNumber
     );
void addToParentSet(uint8_t neighborIdx);
bool neighbors_isParentSetFull(void);
void neighbors_timer_cleanup_cb(opentimer_id_t id);
void neighbors_timer_cleanup_task(void);
void neighbors_timer_cleanup_update(bool shouldRestart);

//=========================== public ==========================================

/**
\brief Initializes this module.
*/
void neighbors_init() {
   uint8_t i;
   
   // clear module variables
   memset(&neighbors_vars,0,sizeof(neighbors_vars_t));
   for (i=0;i<PARENTSETSIZE;i++) {
      neighbors_vars.parents[i] = MAXNUMNEIGHBORS;
   }
   neighbors_vars.cleanupDelay = (uint32_t)DESYNCTIMEOUT * PORT_TsSlotDuration * 2;
   
   // set myDAGrank
   if (idmanager_getIsDAGroot()==TRUE) {
      neighbors_vars.myDAGrank=MINHOPRANKINCREASE;
      leds_sync_on();
   } else {
      neighbors_vars.myDAGrank=DEFAULTDAGRANK;
      neighbors_vars.timerId  = opentimers_start(
                                    neighbors_vars.cleanupDelay,
                                    TIMER_PERIODIC,
                                    TIME_TICS,
                                    neighbors_timer_cleanup_cb
                                 );
   }
   neighbors_vars.myLowestDAGrank = neighbors_vars.myDAGrank;
}

//===== getters

/**
\brief Retrieve this mote's current DAG rank.

\returns This mote's current DAG rank.
*/
dagrank_t neighbors_getMyDAGrank() {
   return neighbors_vars.myDAGrank;
}

/**
\brief Retrieve this mote's current DAG rank.

\returns This mote's lowest DAG rank.
*/
dagrank_t neighbors_getMyLowestDAGrank() {
   return neighbors_vars.myLowestDAGrank;
}

/**
\brief Retrieve the number of neighbors this mote's currently knows of.

\returns The number of neighbors this mote's currently knows of.
*/
uint8_t neighbors_getNumNeighbors() {
   uint8_t i;
   uint8_t returnVal;
   
   returnVal=0;
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (neighbors_vars.neighbors[i].used==TRUE) {
         returnVal++;
      }
   }
   return returnVal;
}

/**
\brief Retrieve my preferred parent's EUI64 address.

\param[out] addressToWrite Where to write the preferred parent's address to.
*/
bool neighbors_getPreferredParentEui64(open_addr_t* addressToWrite) {
   bool              foundPreferred;
   uint8_t           preferredParentIndex;
   
   addressToWrite->type = ADDR_NONE;
   
   foundPreferred       = FALSE;
   preferredParentIndex = neighbors_vars.parents[0];
   
   // Try to find preferred parent
   if (preferredParentIndex != MAXNUMNEIGHBORS) {
      memcpy(addressToWrite,&(neighbors_vars.neighbors[preferredParentIndex].addr_64b),sizeof(open_addr_t));
      addressToWrite->type=ADDR_64B;
      foundPreferred=TRUE;
   }
   
   return foundPreferred;
}

/**
\brief Retrieve a parent from the parent set.

\param[in] addressToWrite Where to write the preferred parent's address to.
\param[in] parentIdx Index of the parent in the parent set
*/
bool neighbors_getParentByIndex(open_addr_t* addressToWrite, uint8_t parentIdx) {
   bool              foundParent;
   uint8_t           neighborIdx;
   
   addressToWrite->type = ADDR_NONE;
   
   foundParent          = FALSE;
   neighborIdx          = neighbors_vars.parents[parentIdx];
   
   // Try to find preferred parent
   if (neighborIdx != MAXNUMNEIGHBORS) {
      memcpy(addressToWrite,&(neighbors_vars.neighbors[neighborIdx].addr_64b),sizeof(open_addr_t));
      addressToWrite->type=ADDR_64B;
      foundParent=TRUE;
   }
   
   return foundParent;
}

/**
\brief Retrieve a parent from the parent set.

\param[in] addressToWrite Where to write the preferred parent's address to.
\param[in] parentIdx Index of the parent in the parent set
*/
void neighbors_getNextHopParent(open_addr_t* addressToWrite) {
   uint8_t           currentParentIndex;
   
   addressToWrite->type = ADDR_NONE;
   
   currentParentIndex = neighbors_vars.currentParentIndex;
   while (TRUE) {
      if (neighbors_vars.parents[currentParentIndex] == MAXNUMNEIGHBORS) {
         currentParentIndex = (currentParentIndex + 1) % PARENTSETSIZE;
         if (currentParentIndex == neighbors_vars.currentParentIndex) {
            break;
         }
      } else {
         memcpy(addressToWrite,&(neighbors_vars.neighbors[neighbors_vars.parents[currentParentIndex]].addr_64b),sizeof(open_addr_t));
         addressToWrite->type=ADDR_64B;
         neighbors_vars.currentParentIndex = (currentParentIndex + 1) % PARENTSETSIZE;
         break;
      }
   }
}

/**
\brief Find neighbor to which to send KA.

This function iterates through the neighbor table and identifies the neighbor
we need to send a KA to, if any. This neighbor satisfies the following
conditions:
- it is one of our preferred parents
- we haven't heard it for over kaPeriod

\param[in] kaPeriod The maximum number of slots I'm allowed not to have heard
   it.

\returns A pointer to the neighbor's address, or NULL if no KA is needed.
*/
open_addr_t* neighbors_getKANeighbor(uint16_t kaPeriod) {
   uint16_t          timeSinceHeard;
   open_addr_t*      addrToKA;
   uint8_t           parentIndex;
   uint8_t           neighborIndex;
   uint8_t           kaIndex;
   dagrank_t         minimumDAGrank;
   
   kaIndex = MAXNUMNEIGHBORS;
   for (parentIndex=0;parentIndex<PARENTSETSIZE;parentIndex++) {
      neighborIndex = neighbors_vars.parents[parentIndex];
      if (neighborIndex != MAXNUMNEIGHBORS) {
         if (openqueue_sixtopGetNumPacketsToNeighbor(&(neighbors_vars.neighbors[neighborIndex].addr_64b))==0) {
            timeSinceHeard = ieee154e_asnDiff(&neighbors_vars.neighbors[neighborIndex].asn);
            if (timeSinceHeard>kaPeriod) {
               kaIndex = neighborIndex;
            }
         }
      }
   }
   
   if (kaIndex == MAXNUMNEIGHBORS) {
      neighborIndex = neighbors_getLowestJoinPriorityNeighborIndex();
      if (
            (neighborIndex!=MAXNUMNEIGHBORS)
            &&
            (neighbors_vars.neighbors[neighborIndex].parentPreference == 0)
            &&
            (
               (
                  (neighbors_vars.neighbors[neighborIndex].DAGrank!=DEFAULTDAGRANK)
                  &&
                  (neighbors_vars.neighbors[neighborIndex].DAGrank < neighbors_vars.myDAGrank)
               )
               ||
               (
                  (neighbors_vars.neighbors[neighborIndex].DAGrank==DEFAULTDAGRANK)
                  &&
                  ((neighbors_vars.neighbors[neighborIndex].joinPrio + 1) * MINHOPRANKINCREASE < neighbors_vars.myDAGrank)
               )
            )
         ) {
         if (openqueue_sixtopGetNumPacketsToNeighbor(&(neighbors_vars.neighbors[neighborIndex].addr_64b))==0) {
            timeSinceHeard = ieee154e_asnDiff(&neighbors_vars.neighbors[neighborIndex].asn);
            if (timeSinceHeard>kaPeriod) {
               kaIndex = neighborIndex;
            }
         }
      }
      if (kaIndex == MAXNUMNEIGHBORS) {
         minimumDAGrank = DEFAULTDAGRANK;
         for (neighborIndex=0;neighborIndex<MAXNUMNEIGHBORS;neighborIndex++) {
            if (
                  (neighbors_vars.neighbors[neighborIndex].used==TRUE)
                  &&
                  (neighbors_vars.neighbors[neighborIndex].parentPreference == 0)
                  &&
                  (neighbors_vars.neighbors[neighborIndex].DAGrank < minimumDAGrank)
                  &&
                  (neighbors_vars.neighbors[neighborIndex].DAGrank < neighbors_vars.myDAGrank)
               ) {
               minimumDAGrank = neighbors_vars.neighbors[neighborIndex].DAGrank;
               kaIndex = neighborIndex;
            }
         }
         if (kaIndex!=MAXNUMNEIGHBORS) {
            if (openqueue_sixtopGetNumPacketsToNeighbor(&(neighbors_vars.neighbors[kaIndex].addr_64b))==0) {
               timeSinceHeard = ieee154e_asnDiff(&neighbors_vars.neighbors[kaIndex].asn);
               if (timeSinceHeard<=kaPeriod) {
                  kaIndex = MAXNUMNEIGHBORS;
               }
            }
         }
      }
   }
   // initialize
   addrToKA = NULL;
   if (kaIndex != MAXNUMNEIGHBORS) {
      addrToKA = &(neighbors_vars.neighbors[kaIndex].addr_64b);
      //printf("%x keepaliving %x\n",idmanager_getMyID(ADDR_64B)->addr_64b[7],addrToKA->addr_64b[7]);
   }
   
   return addrToKA;
}

bool neighbors_getOTFstatisticsByIndex(
      uint8_t        neighborIndex,
      open_addr_t*   address,
      uint8_t*       requiredVirtualCells,
      uint8_t*       aveRequiredVirtualCells
   ) {
   bool returnVal;
   
   returnVal = FALSE;
   if ((neighborIndex<MAXNUMNEIGHBORS) && (neighbors_vars.neighbors[neighborIndex].used==TRUE)) {
      memcpy(
         address,
         &neighbors_vars.neighbors[neighborIndex].addr_64b,
         sizeof(open_addr_t)
      );
      *requiredVirtualCells = neighbors_vars.neighbors[neighborIndex].requiredVirtualCells;
      *aveRequiredVirtualCells = neighbors_vars.neighbors[neighborIndex].aveRequiredVirtualCells;
      neighbors_vars.neighbors[neighborIndex].aveRequiredVirtualCells =
               (3 * neighbors_vars.neighbors[neighborIndex].aveRequiredVirtualCells 
                        + neighbors_vars.neighbors[neighborIndex].requiredVirtualCells) / 4;
      returnVal = TRUE;
   }
   
   return returnVal;
}


//===== interrogators

/**
\brief Indicate whether some neighbor is a stable neighbor

\param[in] address The address of the neighbor, a full 128-bit IPv6 addres.

\returns TRUE if that neighbor is stable, FALSE otherwise.
*/
bool neighbors_isStableNeighbor(open_addr_t* address) {
   uint8_t     i;
   open_addr_t temp_addr_64b;
   open_addr_t temp_prefix;
   bool        returnVal;
   
   // by default, not stable
   returnVal  = FALSE;
   
   // but neighbor's IPv6 address in prefix and EUI64
   switch (address->type) {
      case ADDR_128B:
         packetfunctions_ip128bToMac64b(address,&temp_prefix,&temp_addr_64b);
         break;
      default:
         openserial_printCritical(COMPONENT_NEIGHBORS,ERR_WRONG_ADDR_TYPE,
                               (errorparameter_t)address->type,
                               (errorparameter_t)0);
         return returnVal;
   }
   
   i = findNeighborRow(&temp_addr_64b);
   if (i<MAXNUMNEIGHBORS && neighbors_vars.neighbors[i].stableNeighbor==TRUE) {
      returnVal  = TRUE;
   }
   
   return returnVal;
}

/**
\brief Indicate whether some neighbor is a preferred neighbor.

\param[in] address The EUI64 address of the neighbor.

\returns TRUE if that neighbor is preferred, FALSE otherwise.
*/
bool neighbors_isPreferredParent(open_addr_t* address) {
   uint8_t           i;
   
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   
   i = findNeighborRow(address);
   if (
         (i!=MAXNUMNEIGHBORS)
         &&
         (
            (neighbors_vars.neighbors[i].parentPreference != 0)
            ||
            (i==neighbors_getLowestJoinPriorityNeighborIndex())
         )
      ) {
      ENABLE_INTERRUPTS();
      return TRUE;
   }
   
   ENABLE_INTERRUPTS();
   return FALSE;
}

/**
\brief Indicate whether some neighbor is a parent.

\param[in] address The EUI64 address of the neighbor.

\returns TRUE if that neighbor is parent, FALSE otherwise.
*/
bool neighbors_isParent(open_addr_t* address) {
   bool              returnVal;
   uint8_t           i;
   
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   
   // by default, not preferred
   returnVal = FALSE;
   
   for (i=0;i<PARENTSETSIZE;i++) {
      if (
            (neighbors_vars.parents[i] != MAXNUMNEIGHBORS) 
            &&
            (packetfunctions_sameAddress(address,&neighbors_vars.neighbors[neighbors_vars.parents[i]].addr_64b)==TRUE)
      ) {
         returnVal  = TRUE;
         break;
      }
   }
   
   ENABLE_INTERRUPTS();
   return returnVal;
}

/**
\brief Indicate whether some neighbor has a lower DAG rank that me.

\param[in] index The index of that neighbor in the neighbor table.

\returns TRUE if that neighbor has a lower DAG rank than me, FALSE otherwise.
*/
bool neighbors_isNeighborWithLowerDAGrank(uint8_t index) {
   bool    returnVal;
   
   if (neighbors_vars.neighbors[index].used==TRUE &&
       neighbors_vars.neighbors[index].DAGrank < neighbors_getMyDAGrank()) { 
      returnVal = TRUE;
   } else {
      returnVal = FALSE;
   }
   
   return returnVal;
}

/**
\brief Indicate whether some neighbor has a lower DAG rank that me.

\param[in] index The index of that neighbor in the neighbor table.

\returns TRUE if that neighbor has a lower DAG rank than me, FALSE otherwise.
*/
bool neighbors_isNeighborWithHigherDAGrank(uint8_t index) {
   bool    returnVal;
   
   if (neighbors_vars.neighbors[index].used==TRUE &&
       neighbors_vars.neighbors[index].DAGrank >= neighbors_getMyDAGrank()) { 
      returnVal = TRUE;
   } else {
      returnVal = FALSE;
   }
   
   return returnVal;
}

//===== updating neighbor information

/**
\brief Indicate some (non-ACK) packet was received from a neighbor.

This function should be called for each received (non-ACK) packet so neighbor
statistics in the neighbor table can be updated.

The fields which are updated are:
- numRx
- rssi
- asn
- stableNeighbor
- switchStabilityCounter

\param[in] l2_src MAC source address of the packet, i.e. the neighbor who sent
   the packet just received.
\param[in] rssi   RSSI with which this packet was received.
\param[in] asnTs  ASN at which this packet was received.
\param[in] joinPrioPresent Whether a join priority was present in the received
   packet.
\param[in] joinPrio The join priority present in the packet, if any.
*/
void neighbors_indicateRx(open_addr_t* l2_src,
                          int8_t       rssi,
                          asn_t*       asnTs,
                          bool         joinPrioPresent,
                          uint8_t      joinPrio) {
   uint8_t i;
   
   i = findNeighborRow(l2_src);
   if (i<MAXNUMNEIGHBORS) {
      // update existing neighbor
         
      // update numRx, rssi, asn
      neighbors_vars.neighbors[i].numRx++;
      neighbors_vars.neighbors[i].rssi=rssi;
      memcpy(&neighbors_vars.neighbors[i].asn,asnTs,sizeof(asn_t));
      
      //update jp
      if (joinPrioPresent==TRUE){
         neighbors_vars.neighbors[i].joinPrio=joinPrio;
      }
      
      // update stableNeighbor, switchStabilityCounter
      if (neighbors_vars.neighbors[i].stableNeighbor==FALSE) {
         if (neighbors_vars.neighbors[i].rssi>BADNEIGHBORMAXRSSI) {
            neighbors_vars.neighbors[i].switchStabilityCounter++;
            if (neighbors_vars.neighbors[i].switchStabilityCounter>=SWITCHSTABILITYTHRESHOLD) {
               neighbors_vars.neighbors[i].switchStabilityCounter=0;
               neighbors_vars.neighbors[i].stableNeighbor=TRUE;
            }
         } else {
            neighbors_vars.neighbors[i].switchStabilityCounter=0;
         }
      } else {
         if (neighbors_vars.neighbors[i].rssi<GOODNEIGHBORMINRSSI) {
            neighbors_vars.neighbors[i].switchStabilityCounter++;
            if (neighbors_vars.neighbors[i].switchStabilityCounter>=SWITCHSTABILITYTHRESHOLD) {
               neighbors_vars.neighbors[i].switchStabilityCounter=0;
               neighbors_vars.neighbors[i].stableNeighbor=FALSE;
            }
         } else {
            neighbors_vars.neighbors[i].switchStabilityCounter=0;
         }
      }
   } else {
      // register new neighbor
      registerNewNeighbor(l2_src, rssi, asnTs, joinPrioPresent,joinPrio);
   }
}

/**
\brief Indicate some packet was sent to some neighbor.

This function should be called for each transmitted (non-ACK) packet so
neighbor statistics in the neighbor table can be updated.

The fields which are updated are:
- numTx
- numTxACK
- asn

\param[in] l2_dest MAC destination address of the packet, i.e. the neighbor
   who I just sent the packet to.
\param[in] numTxAttempts Number of transmission attempts to this neighbor.
\param[in] was_finally_acked TRUE iff the packet was ACK'ed by the neighbor
   on final transmission attempt.
\param[in] asnTs ASN of the last transmission attempt.
*/
void neighbors_indicateTx(open_addr_t* l2_dest,
                          bool succesfullTx,
                          asn_t*       asnTs) {
   uint8_t i;
   
   // don't run through this function if packet was sent to broadcast address
   if (packetfunctions_isBroadcastMulticast(l2_dest)==FALSE) {
      
      i = findNeighborRow(l2_dest);
      if (i<MAXNUMNEIGHBORS) {
         
         // handle roll-over case
         if (neighbors_vars.neighbors[i].numTx==0xff) {
            neighbors_vars.neighbors[i].numWraps++; //counting the number of times that tx wraps.
            neighbors_vars.neighbors[i].numTx/=2;
            neighbors_vars.neighbors[i].numTxACK/=2;
         }
         // update statistics
         neighbors_vars.neighbors[i].numTx++; 
         
         if (succesfullTx==TRUE) {
            neighbors_vars.neighbors[i].numTxACK++;
            memcpy(&neighbors_vars.neighbors[i].asn,asnTs,sizeof(asn_t));
         }
      }
   }
}

/**
\brief Indicate I just received a RPL DIO from a neighbor.

This function should be called for each received a DIO is received so neighbor
routing information in the neighbor table can be updated.

The fields which are updated are:
- DAGrank

\param[in] msg The received message with msg->payload pointing to the DIO
   header.
\param[in] newDODAGID Bool value passed by reference and modified if
*/
void neighbors_indicateRxDIO(OpenQueueEntry_t* msg, bool* newDODAGID) {
   uint8_t          i;
   uint8_t          j;
   uint8_t          temp_8b;
  
   // take ownership over the packet
   msg->owner = COMPONENT_NEIGHBORS;
   
   // update rank of that neighbor in table
   neighbors_vars.dio = (icmpv6rpl_dio_ht*)(msg->payload);
   // retrieve rank
   temp_8b            = *(msg->payload+2);
   neighbors_vars.dio->rank = (temp_8b << 8) + *(msg->payload+3);
   i = findNeighborRow(&(msg->l2_nextORpreviousHop));
   if (i<MAXNUMNEIGHBORS) {
      if (*newDODAGID == FALSE) {
         neighbors_vars.neighbors[i].DAGrank = neighbors_vars.dio->rank;
      } else {
         for (j=0;((j<MAXNUMNEIGHBORS) && (j != i));j++) {
            removeNeighbor(j);
         }
         neighbors_vars.myDAGrank = DEFAULTDAGRANK;
         neighbors_vars.myLowestDAGrank = neighbors_vars.myDAGrank;
         neighbors_vars.neighbors[i].DAGrank = neighbors_vars.dio->rank;
      }
   } else {
      *newDODAGID = FALSE;
   }
   // update my routing information
   neighbors_updateMyDAGrankAndNeighborPreference(); 
}

//===== write addresses

/**
\brief Write the 64-bit address of some neighbor to some location.
*/
void  neighbors_getNeighbor(open_addr_t* address, uint8_t addr_type, uint8_t index){
   switch(addr_type) {
      case ADDR_64B:
         memcpy(&(address->addr_64b),&(neighbors_vars.neighbors[index].addr_64b.addr_64b),LENGTH_ADDR64b);
         address->type=ADDR_64B;
         break;
      default:
         openserial_printCritical(COMPONENT_NEIGHBORS,ERR_WRONG_ADDR_TYPE,
                               (errorparameter_t)addr_type,
                               (errorparameter_t)1);
         break; 
   }
}

//===== setters

void neighbors_setMyDAGrank(dagrank_t rank){
    neighbors_vars.myDAGrank = rank;
}

void neighbors_setAdvertizedDAGrank() {
   if (neighbors_vars.myDAGrank < neighbors_vars.myLowestDAGrank) {
      neighbors_vars.myLowestDAGrank = neighbors_vars.myDAGrank;
   }
}

void neighbors_updateEnqueuedPacketsToNeighbor(open_addr_t* address) {
   uint8_t i;
      
   i = findNeighborRow(address);
   if (i != MAXNUMNEIGHBORS) {
      neighbors_vars.neighbors[i].requiredVirtualCells++;
   }
}

void neighbors_resetOTFstatistics() {
   uint8_t i;
   
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (neighbors_vars.neighbors[i].used==TRUE) {
         neighbors_vars.neighbors[i].requiredVirtualCells = 
                  openqueue_sixtopGetNumPacketsToNeighbor(&(neighbors_vars.neighbors[i].addr_64b));
      }
   }
}

//===== managing routing info

/**
\brief Update my DAG rank and neighbor preference.

Call this function whenever some data is changed that could cause this mote's
routing decisions to change. Examples are:
- I received a DIO which updated by neighbor table. If this DIO indicated a
  very low DAGrank, I may want to change by routing parent.
- I became a DAGroot, so my DAGrank should be 0.
*/
void neighbors_updateMyDAGrankAndNeighborPreference() {
   uint8_t     i;
   uint8_t     j;
   dagrank_t   rankIncrease;
   uint32_t    tentativeDAGrank; // 32-bit since is used to sum
   uint32_t    rankIncreaseIntermediary; // stores intermediary results of rankIncrease calculation
   uint8_t     tempParent;
   uint16_t    useThreshold;
   dagrank_t   finalDAGrank;
   dagrank_t   tempDAGrank;
   
   // if I'm a DAGroot, my DAGrank is always MINHOPRANKINCREASE
   if ((idmanager_getIsDAGroot())==TRUE) {
       // the dagrank is not set through setting command, set rank to MINHOPRANKINCREASE here 
       neighbors_vars.myDAGrank=MINHOPRANKINCREASE;
       neighbors_vars.myLowestDAGrank = neighbors_vars.myDAGrank;
       neighbors_timer_cleanup_update(FALSE);
       leds_sync_on();
       return;
   }
   
   // STEP 1: loop through neighbor table, reset preference (will be updated), calculate resulting DAGrank
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (neighbors_vars.neighbors[i].used==TRUE) {
         
         neighbors_vars.neighbors[i].parentPreference = 0;
         
         // calculate link cost to this neighbor
         if (neighbors_vars.neighbors[i].numTxACK==0) {
            rankIncrease = DEFAULTLINKCOST*2*MINHOPRANKINCREASE;
         } else {
            //6TiSCH minimal draft using OF0 for rank computation
            rankIncreaseIntermediary = (((uint32_t)neighbors_vars.neighbors[i].numTx) << 10);
            rankIncreaseIntermediary = (rankIncreaseIntermediary * 2 * MINHOPRANKINCREASE) / 
                     ((uint32_t)neighbors_vars.neighbors[i].numTxACK);
            rankIncrease = (uint16_t)(rankIncreaseIntermediary >> 10);
            if (neighbors_vars.neighbors[i].numTx < DEFAULTLINKCOST/2) {
               rankIncrease += (DEFAULTLINKCOST/2 - neighbors_vars.neighbors[i].numTx) * 2 * MINHOPRANKINCREASE;
            }
         }
         // if the resulting DAGrank is bigger than MAXDAGRANK, set the resulting DAGrank to MAXDAGRANK
         tentativeDAGrank = neighbors_vars.neighbors[i].DAGrank+rankIncrease;
         if (tentativeDAGrank > MAXDAGRANK) {
            tentativeDAGrank = MAXDAGRANK;
         }
         // update the resultingDAGrank for this neighbor
         neighbors_vars.neighbors[i].resultingDAGrank = tentativeDAGrank;
      }
   }
   
   // STEP 2: sort and clean the current parent set according to the resulting DAGrank
   for (i=0;i<PARENTSETSIZE;i++) {
      for (j=i+1;j<PARENTSETSIZE;j++) {
         if (neighbors_vars.parents[i] == MAXNUMNEIGHBORS) {
            neighbors_vars.parents[i] = neighbors_vars.parents[j];
            neighbors_vars.parents[j] = MAXNUMNEIGHBORS;
         } else if (neighbors_vars.parents[j] != MAXNUMNEIGHBORS) {
            if (neighbors_vars.neighbors[neighbors_vars.parents[i]].resultingDAGrank > 
                  neighbors_vars.neighbors[neighbors_vars.parents[j]].resultingDAGrank) {
               tempParent = neighbors_vars.parents[i];
               neighbors_vars.parents[i] = neighbors_vars.parents[j];
               neighbors_vars.parents[j] = tempParent;
            }
         }
      }
      if (neighbors_vars.parents[i] == MAXNUMNEIGHBORS) {
         break;
      } else if (
            (neighbors_vars.neighbors[neighbors_vars.parents[i]].resultingDAGrank == MAXDAGRANK)
            ||
            (
               ((uint32_t)neighbors_vars.myLowestDAGrank + MAXRANKINCREASE < MAXDAGRANK)
               &&
               (neighbors_vars.neighbors[neighbors_vars.parents[i]].resultingDAGrank >
                  neighbors_vars.myLowestDAGrank + MAXRANKINCREASE) 
            )
         ) {
         for (j=i;j<PARENTSETSIZE;j++) {
            neighbors_vars.parents[j] = MAXNUMNEIGHBORS;
         }
         break;
      }
   }
   
   // STEP 3: check if the current parent set is alredy full, if yes enable use of PARENTSWITCHTHRESHOLD
   useThreshold = 0;
   if (neighbors_vars.parents[PARENTSETSIZE-1] != MAXNUMNEIGHBORS) {
      useThreshold = 1;
   }
   
   // STEP 4: loop through neighbors and check if they can be added to the parent set
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (
            (neighbors_vars.neighbors[i].used==TRUE)
            &&
            (neighbors_vars.neighbors[i].resultingDAGrank != MAXDAGRANK)
            &&
            (
               ((uint32_t)neighbors_vars.myLowestDAGrank + MAXRANKINCREASE >= MAXDAGRANK)
               ||
               (
                  ((uint32_t)neighbors_vars.myLowestDAGrank + MAXRANKINCREASE < MAXDAGRANK)
                  &&
                  (neighbors_vars.neighbors[i].resultingDAGrank <= neighbors_vars.myLowestDAGrank + MAXRANKINCREASE)
               )
            )
            &&
            (
               (neighbors_vars.parents[PARENTSETSIZE-1] == MAXNUMNEIGHBORS)
               ||
               (
                  (neighbors_vars.parents[PARENTSETSIZE-1] != MAXNUMNEIGHBORS)
                  &&
                  ((uint32_t)neighbors_vars.neighbors[i].resultingDAGrank + (uint32_t)useThreshold * PARENTSWITCHTHRESHOLD <
                     MAXDAGRANK)
                  &&
                  (neighbors_vars.neighbors[neighbors_vars.parents[PARENTSETSIZE-1]].resultingDAGrank >= 
                     neighbors_vars.neighbors[i].resultingDAGrank + useThreshold * PARENTSWITCHTHRESHOLD)
               )
            ) 
         ) {
         addToParentSet(i);
      }
   }
   
   // STEP 5: set preferences
   for (j=0;j<PARENTSETSIZE;j++) {
      if (neighbors_vars.parents[j] != MAXNUMNEIGHBORS) {
         if (j==0) {
            neighbors_vars.neighbors[neighbors_vars.parents[j]].parentPreference       = MAXPREFERENCE;
         } else {
            neighbors_vars.neighbors[neighbors_vars.parents[j]].parentPreference       = INTERMEDIATEPREFERENCE;
         }
         neighbors_vars.neighbors[neighbors_vars.parents[j]].stableNeighbor            = TRUE;
         neighbors_vars.neighbors[neighbors_vars.parents[j]].switchStabilityCounter    = 0;
      } else {
         break;
      }
   }
   
   // STEP 6: update dagrank
   finalDAGrank = DEFAULTDAGRANK;
   if (neighbors_vars.parents[0] != MAXNUMNEIGHBORS) {
      finalDAGrank = neighbors_vars.neighbors[neighbors_vars.parents[0]].resultingDAGrank;
      for (j=0;j<PARENTSETSIZE;j++) {
         if (neighbors_vars.parents[j] != MAXNUMNEIGHBORS) {
            tempDAGrank = 
                     ((neighbors_vars.neighbors[neighbors_vars.parents[j]].DAGrank / MINHOPRANKINCREASE) + 1) * MINHOPRANKINCREASE;
            if (tempDAGrank > finalDAGrank) {
               finalDAGrank = tempDAGrank;
            }
         } else {
            break;
         }
      }
      for (j=0;j<PARENTSETSIZE;j++) {
         if (neighbors_vars.parents[j] != MAXNUMNEIGHBORS) {
            if (neighbors_vars.neighbors[neighbors_vars.parents[j]].resultingDAGrank > MAXRANKINCREASE) {
               tempDAGrank = 
                        neighbors_vars.neighbors[neighbors_vars.parents[j]].resultingDAGrank - MAXRANKINCREASE;
               if (tempDAGrank > finalDAGrank) {
                  finalDAGrank = tempDAGrank;
               }
            }
         } else {
            break;
         }
      }
   }
   neighbors_vars.myDAGrank = finalDAGrank;
   if (finalDAGrank != DEFAULTDAGRANK) {
      neighbors_timer_cleanup_update(TRUE);
      leds_sync_on();
   } else {
      leds_sync_off();
   }
}

//===== maintenance

void  neighbors_removeOld() {
   uint8_t    i;
   uint16_t   timeSinceHeard;
   
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (neighbors_vars.neighbors[i].used==TRUE) {
         timeSinceHeard = ieee154e_asnDiff(&neighbors_vars.neighbors[i].asn);
         if (timeSinceHeard>DESYNCTIMEOUT) {
            removeNeighbor(i);
         }
      }
   }
   // update my routing information
   neighbors_updateMyDAGrankAndNeighborPreference();
}

//===== debug

/**
\brief Triggers this module to print status information, over serial.

debugPrint_* functions are used by the openserial module to continuously print
status information about several modules in the OpenWSN stack.

\returns TRUE if this function printed something, FALSE otherwise.
*/
bool debugPrint_neighbors() {
   debugNeighborEntry_t temp;
   
   neighbors_vars.debugRow=(neighbors_vars.debugRow+1)%MAXNUMNEIGHBORS;
   
   temp.row                            = \
      neighbors_vars.debugRow;
   temp.used                           = \
      neighbors_vars.neighbors[neighbors_vars.debugRow].used;
   temp.parentPreference               = \
      neighbors_vars.neighbors[neighbors_vars.debugRow].parentPreference;
   temp.stableNeighbor                 = \
      neighbors_vars.neighbors[neighbors_vars.debugRow].stableNeighbor;
   temp.switchStabilityCounter         = \
      neighbors_vars.neighbors[neighbors_vars.debugRow].switchStabilityCounter;
   memcpy(
      &temp.addr_64b,
      &neighbors_vars.neighbors[neighbors_vars.debugRow].addr_64b,
      sizeof(open_addr_t)
   );
   temp.DAGrank                        = \
      neighbors_vars.neighbors[neighbors_vars.debugRow].DAGrank;
   temp.rssi                           = \
      neighbors_vars.neighbors[neighbors_vars.debugRow].rssi;
   temp.numRx                          = \
      neighbors_vars.neighbors[neighbors_vars.debugRow].numRx;
   temp.numTx                          = \
      neighbors_vars.neighbors[neighbors_vars.debugRow].numTx;
   temp.numTxACK                       = \
      neighbors_vars.neighbors[neighbors_vars.debugRow].numTxACK;
   temp.numWraps                       = \
      neighbors_vars.neighbors[neighbors_vars.debugRow].numWraps;
   memcpy(
      &temp.asn,
      &neighbors_vars.neighbors[neighbors_vars.debugRow].asn,
      sizeof(asn_t)
   );
   temp.joinPrio                       = \
      neighbors_vars.neighbors[neighbors_vars.debugRow].joinPrio;
   
   openserial_printStatus(STATUS_NEIGHBORS,(uint8_t*)&temp,sizeof(debugNeighborEntry_t));
   
   return TRUE;
}

//=========================== private =========================================

void registerNewNeighbor(open_addr_t* address,
                         int8_t       rssi,
                         asn_t*       asnTimestamp,
                         bool         joinPrioPresent,
                         uint8_t      joinPrio) {
   uint8_t     i,j;
   bool        f_isUsedRow;
   bool        f_isUnstableNeighborAvailable;
   dagrank_t   maxDAGrank;
   
   // filter errors
   if (address->type!=ADDR_64B) {
      openserial_printCritical(COMPONENT_NEIGHBORS,ERR_WRONG_ADDR_TYPE,
                            (errorparameter_t)address->type,
                            (errorparameter_t)2);
      return;
   }
   
   i = MAXNUMNEIGHBORS;
   f_isUsedRow = TRUE;
   f_isUnstableNeighborAvailable = FALSE;
   maxDAGrank = 0;
   for (j=0;j<MAXNUMNEIGHBORS;j++) {
      if (neighbors_vars.neighbors[j].used == FALSE) {
         i = j;
         f_isUsedRow = FALSE;
         break;
      } else {
         if (neighbors_vars.neighbors[j].stableNeighbor == FALSE) {
            if (f_isUnstableNeighborAvailable == FALSE) {
               f_isUnstableNeighborAvailable = TRUE;
               maxDAGrank = 0;
            }
            if (neighbors_vars.neighbors[j].DAGrank>maxDAGrank) {
               i = j;
               maxDAGrank = neighbors_vars.neighbors[j].DAGrank;
            }
         } else {
            if (f_isUnstableNeighborAvailable == FALSE) {
               if (
                  (neighbors_vars.neighbors[j].DAGrank > neighbors_vars.myDAGrank)
                  &&
                  (neighbors_vars.neighbors[j].DAGrank > maxDAGrank)
               ) {
                  i = j;
                  maxDAGrank = neighbors_vars.neighbors[j].DAGrank;
               }
            }
         }
      }
   }
   
   if (i<MAXNUMNEIGHBORS) {
      if (f_isUsedRow) {
         removeNeighbor(i);
      }
      // add this neighbor
      neighbors_vars.neighbors[i].used                      = TRUE;
      neighbors_vars.neighbors[i].parentPreference          = 0;
      // neighbors_vars.neighbors[i].stableNeighbor         = FALSE;
      // Note: all new neighbors are consider stable
      neighbors_vars.neighbors[i].stableNeighbor            = TRUE;
      neighbors_vars.neighbors[i].switchStabilityCounter    = 0;
      memcpy(&neighbors_vars.neighbors[i].addr_64b,address,sizeof(open_addr_t));
      neighbors_vars.neighbors[i].DAGrank                   = DEFAULTDAGRANK;
      neighbors_vars.neighbors[i].resultingDAGrank          = DEFAULTDAGRANK;
      neighbors_vars.neighbors[i].rssi                      = rssi;
      neighbors_vars.neighbors[i].numRx                     = 1;
      neighbors_vars.neighbors[i].numTx                     = 0;
      neighbors_vars.neighbors[i].numTxACK                  = 0;
      memcpy(&neighbors_vars.neighbors[i].asn,asnTimestamp,sizeof(asn_t));
      //update jp
      if (joinPrioPresent==TRUE){
         neighbors_vars.neighbors[i].joinPrio=joinPrio;
      } else {
         neighbors_vars.neighbors[i].joinPrio=DEFAULTJOINPRIORITY;
      }
      neighbors_vars.neighbors[i].requiredVirtualCells = 
         openqueue_sixtopGetNumPacketsToNeighbor(&(neighbors_vars.neighbors[i].addr_64b));
      neighbors_vars.neighbors[i].aveRequiredVirtualCells = 
         neighbors_vars.neighbors[i].requiredVirtualCells;
   } else {
      openserial_printError(COMPONENT_NEIGHBORS,ERR_NEIGHBORS_FULL,
                            (errorparameter_t)MAXNUMNEIGHBORS,
                            (errorparameter_t)0);
   }
}

uint8_t findNeighborRow(open_addr_t* neighbor) {
   uint8_t i=0;
   while (i<MAXNUMNEIGHBORS) {
      if (isThisRowMatching(neighbor,i)) {
         break;
      }
      i++;
   }
   return i;
}

void removeNeighbor(uint8_t neighborIndex) {
   uint8_t i;
   
   for (i=0;i<PARENTSETSIZE;i++) {
      if (neighbors_vars.parents[i] == neighborIndex) {
         neighbors_vars.parents[i] = MAXNUMNEIGHBORS;
         break;
      }
   }
   memset(&(neighbors_vars.neighbors[neighborIndex]),0,sizeof(neighborRow_t));
   neighbors_vars.neighbors[neighborIndex].DAGrank                   = DEFAULTDAGRANK;
   neighbors_vars.neighbors[neighborIndex].resultingDAGrank          = DEFAULTDAGRANK;
   neighbors_vars.neighbors[neighborIndex].joinPrio                  = DEFAULTJOINPRIORITY;
}

uint8_t neighbors_getLowestJoinPriorityNeighborIndex() {
   uint8_t           i;
   uint8_t           lowestJoinPriority;
   uint8_t           lowestJoinPriorityNeighborIndex;
   
   lowestJoinPriority = DEFAULTJOINPRIORITY;
   lowestJoinPriorityNeighborIndex = MAXNUMNEIGHBORS;
   
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (
            (neighbors_vars.neighbors[i].used==TRUE)
            &&
            (neighbors_vars.neighbors[i].joinPrio < lowestJoinPriority)
         ) {
         lowestJoinPriority = neighbors_vars.neighbors[i].joinPrio;
         lowestJoinPriorityNeighborIndex = i;
      }
   }
   
   return lowestJoinPriorityNeighborIndex;
}

//=========================== helpers =========================================

bool isThisRowMatching(open_addr_t* address, uint8_t rowNumber) {
   switch (address->type) {
      case ADDR_64B:
         return neighbors_vars.neighbors[rowNumber].used &&
                packetfunctions_sameAddress(address,&neighbors_vars.neighbors[rowNumber].addr_64b);
      default:
         openserial_printCritical(COMPONENT_NEIGHBORS,ERR_WRONG_ADDR_TYPE,
                               (errorparameter_t)address->type,
                               (errorparameter_t)3);
         return FALSE;
   }
}

void addToParentSet(uint8_t neighborIdx) {
   uint8_t     parentIdx;
   
   //assuming that the parent set is sorted according to ranks
   
   // check if neighbor is already in parent set
   for (parentIdx=0;parentIdx<PARENTSETSIZE;parentIdx++) {
      if (neighbors_vars.parents[parentIdx] == neighborIdx) {
         return;
      } else if (neighbors_vars.parents[parentIdx] == MAXNUMNEIGHBORS) {
         break;
      }
   }
   
   // if the parent set is full, delete the last one
   if (parentIdx == PARENTSETSIZE) {
      parentIdx--;
      neighbors_vars.parents[parentIdx] = MAXNUMNEIGHBORS;
   }
   
   // add the parent index in the right position
   while (TRUE) {
      // has the last parent in the parent set a DAGrank greater than that of the neighbor 
      if (
            (parentIdx!=0)
            &&
            (neighbors_vars.neighbors[neighbors_vars.parents[parentIdx-1]].resultingDAGrank > 
               neighbors_vars.neighbors[neighborIdx].resultingDAGrank)
         ) {
         neighbors_vars.parents[parentIdx] = neighbors_vars.parents[parentIdx-1];
         parentIdx--;
      } else {
         neighbors_vars.parents[parentIdx] = neighborIdx;
         break;
      }
   }
}

bool neighbors_isParentSetFull() {
   bool returnValue;
   uint8_t     parentIdx;
   
   returnValue = TRUE;
   for (parentIdx=0;parentIdx<PARENTSETSIZE;parentIdx++) {
      if (neighbors_vars.parents[parentIdx] == MAXNUMNEIGHBORS) {
         returnValue = FALSE;
         break;
      }
   }
   return returnValue;
}

void neighbors_timer_cleanup_cb(opentimer_id_t id) {
   scheduler_push_task(neighbors_timer_cleanup_task,TASKPRIO_RPL);
}

void neighbors_timer_cleanup_task(void) {
   neighbors_vars.myLowestDAGrank = neighbors_vars.myDAGrank;
}

void neighbors_timer_cleanup_update(bool shouldRestart) {
   opentimers_stop(neighbors_vars.timerId);
   if (shouldRestart == TRUE) {
      opentimers_setPeriod(
                              neighbors_vars.timerId, 
                              TIME_TICS,
                              neighbors_vars.cleanupDelay
                          );
      opentimers_restart(neighbors_vars.timerId);
   }
}
