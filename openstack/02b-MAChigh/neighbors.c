#include "opendefs.h"
#include "neighbors.h"
#include "openqueue.h"
#include "packetfunctions.h"
#include "idmanager.h"
#include "openserial.h"
#include "IEEE802154E.h"

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
bool isThisRowMatching(
        open_addr_t* address,
        uint8_t      rowNumber
     );
bool neighbors_isSameDODAGID(void);
bool neighbors_isDODAGverNumbHigher(
        DODAGversion_t firstVerNumb, 
        DODAGversion_t secondVerNumb
     );

//=========================== public ==========================================

/**
\brief Initializes this module.
*/
void neighbors_init() {
   
   // clear module variables
   memset(&neighbors_vars,0,sizeof(neighbors_vars_t));
   
   // set myDODAGvesion and myRank
   if (idmanager_getIsDAGroot()==TRUE) {
      neighbors_vars.myDODAGversion=DEFAULTDODAGVERSION;
      neighbors_vars.myRank=0;
   } else {
      neighbors_vars.myDODAGversion=DEFAULTDODAGVERSION;
      neighbors_vars.myRank=DEFAULTDAGRANK;
   }
}

//===== getters

/**
\brief Retrieve this mote's current DAG rank.

\returns This mote's current DAG rank.
*/
rank_t neighbors_getMyDAGrank() {
   return neighbors_vars.myRank;
}

/**
\brief Retrieve this mote's current DODAG version.

\returns This mote's current DODAG version.
*/
DODAGversion_t neighbors_getMyDODAGVersion() {
   return neighbors_vars.myDODAGversion;
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
   uint8_t   i;
   bool      foundPreferred;
   uint8_t   numNeighbors;
   rank_t minRankVal;
   uint8_t   minRankIdx;
   
   addressToWrite->type = ADDR_NONE;
   
   foundPreferred       = FALSE;
   numNeighbors         = 0;
   minRankVal           = MAXDAGRANK;
   minRankIdx           = MAXNUMNEIGHBORS+1;
   
   //===== step 1. Try to find preferred parent
   for (i=0; i<MAXNUMNEIGHBORS; i++) {
      if (neighbors_vars.neighbors[i].used==TRUE){
         if (neighbors_vars.neighbors[i].parentPreference==MAXPREFERENCE) {
            memcpy(addressToWrite,&(neighbors_vars.neighbors[i].addr_64b),sizeof(open_addr_t));
            addressToWrite->type=ADDR_64B;
            foundPreferred=TRUE;
         }
         // identify neighbor with lowest rank
         if (neighbors_vars.neighbors[i].rank < minRankVal) {
            minRankVal=neighbors_vars.neighbors[i].rank;
            minRankIdx=i;
         }
         numNeighbors++;
      }
   }
   
   //===== step 2. (backup) Promote neighbor with min rank to preferred parent
   if (foundPreferred==FALSE && numNeighbors > 0){
      // promote neighbor
      neighbors_vars.neighbors[minRankIdx].parentPreference       = MAXPREFERENCE;
      neighbors_vars.neighbors[minRankIdx].stableNeighbor         = TRUE;
      neighbors_vars.neighbors[minRankIdx].switchStabilityCounter = 0;
      // return its address
      memcpy(addressToWrite,&(neighbors_vars.neighbors[minRankIdx].addr_64b),sizeof(open_addr_t));
      addressToWrite->type=ADDR_64B;
      foundPreferred=TRUE;         
   }
   
   return foundPreferred;
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
   uint8_t         i;
   uint16_t        timeSinceHeard;
   open_addr_t*    addrPreferred;
   open_addr_t*    addrOther;
   
   // initialize
   addrPreferred = NULL;
   addrOther     = NULL;
   
   // scan through the neighbor table, and populate addrPreferred and addrOther
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (neighbors_vars.neighbors[i].used==TRUE) {
         timeSinceHeard = ieee154e_asnDiff(&neighbors_vars.neighbors[i].asn);
         if (timeSinceHeard>kaPeriod) {
            // this neighbor needs to be KA'ed to
            if (neighbors_vars.neighbors[i].parentPreference==MAXPREFERENCE) {
               // its a preferred parent
               addrPreferred = &(neighbors_vars.neighbors[i].addr_64b);
            } else {
               // its not a preferred parent
               // Note: commented out since policy is not to KA to non-preferred parents
               // addrOther =     &(neighbors_vars.neighbors[i].addr_64b);
            }
         }
      }
   }
   
   // return the EUI64 of the most urgent KA to send:
   // - if available, preferred parent
   // - if not, non-preferred parent
   if        (addrPreferred!=NULL) {
      return addrPreferred;
   } else if (addrOther!=NULL) {
      return addrOther;
   } else {
      return NULL;
   }
}

//===== interrogators

/**
\brief Indicate whether the DODAGID has been changed

\returns TRUE if the DODAGID has been changed, FALSE otherwise.
*/
bool neighbors_isDODAGIDchanged(void) {
   bool returnVal;

   returnVal = neighbors_vars.f_changedDODAGID;
   neighbors_vars.f_changedDODAGID = FALSE;

   return returnVal;
}

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
   uint8_t i;
   bool    returnVal;
   
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   
   // by default, not preferred
   returnVal = FALSE;
   
   i = findNeighborRow(address);
   if (i<MAXNUMNEIGHBORS && neighbors_vars.neighbors[i].parentPreference==MAXPREFERENCE) {
      returnVal  = TRUE;
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
       neighbors_vars.neighbors[index].rank < neighbors_getMyDAGrank()) { 
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
       neighbors_vars.neighbors[index].rank >= neighbors_getMyDAGrank()) { 
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
      } else if (neighbors_vars.neighbors[i].stableNeighbor==TRUE) {
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
                          uint8_t      numTxAttempts,
                          bool         was_finally_acked,
                          asn_t*       asnTs) {
   uint8_t i;
   // don't run through this function if packet was sent to broadcast address
   if (packetfunctions_isBroadcastMulticast(l2_dest)==TRUE) {
      return;
   }
   i = findNeighborRow(l2_dest);
   if (i<MAXNUMNEIGHBORS) {
      
      // handle roll-over case
      if (neighbors_vars.neighbors[i].numTx>(0xff-numTxAttempts)) {
         neighbors_vars.neighbors[i].numWraps++; //counting the number of times that tx wraps.
         neighbors_vars.neighbors[i].numTx/=2;
         neighbors_vars.neighbors[i].numTxACK/=2;
      }
      // update statistics
      neighbors_vars.neighbors[i].numTx += numTxAttempts; 
      
      if (was_finally_acked==TRUE) {
         neighbors_vars.neighbors[i].numTxACK++;
         memcpy(&neighbors_vars.neighbors[i].asn,asnTs,sizeof(asn_t));
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
*/
void neighbors_indicateRxDIO(OpenQueueEntry_t* msg) {
   uint8_t          i;
   uint8_t          row;
  
   // take ownership over the packet
   msg->owner = COMPONENT_NEIGHBORS;
   
   // update rank of that neighbor in table
   neighbors_vars.dio = (icmpv6rpl_dio_ht*)(msg->payload);
   i = findNeighborRow(&(msg->l2_nextORpreviousHop));
   if (i<MAXNUMNEIGHBORS) {
      if (neighbors_isSameDODAGID()) {
         
         neighbors_vars.neighbors[i].DODAGversion = neighbors_vars.dio->verNumb;
         if (
               neighbors_vars.dio->rank > neighbors_vars.neighbors[i].rank &&
               neighbors_vars.dio->rank - neighbors_vars.neighbors[i].rank >(DEFAULTLINKCOST*2*MINHOPRANKINCREASE)
            ) {
            // the new DAGrank looks suspiciously high, only increment a bit
            neighbors_vars.neighbors[i].rank += (DEFAULTLINKCOST*2*MINHOPRANKINCREASE);
            openserial_printError(COMPONENT_NEIGHBORS,ERR_LARGE_DAGRANK,
                                  (errorparameter_t)neighbors_vars.dio->rank,
                                  (errorparameter_t)neighbors_vars.neighbors[i].rank);
         } else {
            neighbors_vars.neighbors[i].rank = neighbors_vars.dio->rank;
         }

      } else {
         
         for (row=0;row<MAXNUMNEIGHBORS;row++) {
            if (row != i) {
               removeNeighbor(row);
            }
         }
         neighbors_writeDODAGid(&(neighbors_vars.dio->DODAGID[0]));
         neighbors_vars.f_changedDODAGID = TRUE;
         neighbors_vars.neighbors[i].DODAGversion = neighbors_vars.dio->verNumb;
         neighbors_vars.neighbors[i].rank = neighbors_vars.dio->rank;
         
      }
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

//===== managing routing info

void neighbors_writeDODAGid(uint8_t* dodagid) {
   // write DODAGID to myDODAGID
   memcpy(
      &(neighbors_vars.myDODAGID[0]),
      dodagid,
      sizeof(neighbors_vars.myDODAGID)
   );
}

/**
\brief Update my DAG rank and neighbor preference.

Call this function whenever some data is changed that could cause this mote's
routing decisions to change. Examples are:
- I received a DIO which updated by neighbor table. If this DIO indicated a
  very low DAGrank, I may want to change by routing parent.
- I became a DAGroot, so my DAGrank should be 0.
*/
void neighbors_updateMyDAGrankAndNeighborPreference() {
   uint8_t   i;
   rank_t    rankIncrease;
   uint32_t  tentativeRank; // 32-bit since is used to sum
   uint8_t   prefParentIdx;
   uint8_t   feasParentIdx;
   rank_t    prefParentRank;
   rank_t    feasParentRank;
   
   // if I'm a DAGroot, my DAGrank is always 0
   if ((idmanager_getIsDAGroot())==TRUE) {
      return;
   }
   
   // by default, I haven't found a preferred parent
   prefParentRank             = MAXDAGRANK;
   feasParentRank             = MAXDAGRANK;
   prefParentIdx              = MAXNUMNEIGHBORS;
   feasParentIdx              = MAXNUMNEIGHBORS;
   
   // loop through neighbor table, update myDAGrank
   for (i=0;i<MAXNUMNEIGHBORS;i++) {
      if (neighbors_vars.neighbors[i].used==TRUE) {
         
         // calculate link cost to this neighbor
         if (neighbors_vars.neighbors[i].numTxACK==0) {
            rankIncrease = DEFAULTLINKCOST*2*MINHOPRANKINCREASE;
         } else {
            //6TiSCH minimal draft using OF0 for rank computation
            rankIncrease = (uint16_t)((((float)neighbors_vars.neighbors[i].numTx)/((float)neighbors_vars.neighbors[i].numTxACK))*2*MINHOPRANKINCREASE);
         }
         tentativeRank = neighbors_vars.neighbors[i].rank+rankIncrease;
         if (tentativeRank > MAXDAGRANK) {
            tentativeRank = MAXDAGRANK;
         }
         
         if (neighbors_vars.neighbors[i].parentPreference == MAXPREFERENCE) {
            prefParentRank             = tentativeRank;
            prefParentIdx              = i;
         } else {
            if (tentativeRank < feasParentRank) {
               // found feasible parent, lower feasParentRank
               feasParentRank             = tentativeRank;
               feasParentIdx              = i;
            }
         }
      }
   } 
   
   // update preferred parent
   if (prefParentRank < MAXDAGRANK) {
      if (feasParentRank < MAXDAGRANK) {
         //TODO
      } else {
         neighbors_vars.myDODAGversion                                  = neighbors_vars.neighbors[prefParentIdx].DODAGversion;
         neighbors_vars.myRank                                          = tentativeRank;
         neighbors_vars.neighbors[prefParentIdx].parentPreference       = MAXPREFERENCE;
         neighbors_vars.neighbors[prefParentIdx].stableNeighbor         = TRUE;
         neighbors_vars.neighbors[prefParentIdx].switchStabilityCounter = 0;
      }
   } else {
      if (feasParentRank < MAXDAGRANK) {
         //TODO
      } else {
         neighbors_vars.myRank                                          = MAXDAGRANK;
      } 
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
   temp.row=neighbors_vars.debugRow;
   temp.neighborEntry=neighbors_vars.neighbors[neighbors_vars.debugRow];
   openserial_printStatus(STATUS_NEIGHBORS,(uint8_t*)&temp,sizeof(debugNeighborEntry_t));
   return TRUE;
}

//=========================== private =========================================

void registerNewNeighbor(open_addr_t* address,
                         int8_t       rssi,
                         asn_t*       asnTimestamp,
                         bool         joinPrioPresent,
                         uint8_t      joinPrio) {
   uint8_t  row;
   uint8_t  j;
   bool     iHaveAPreferedParent;
   
   row = 0;
   while (row<MAXNUMNEIGHBORS) {
      if (neighbors_vars.neighbors[row].used==FALSE) {
         break;
      }
      row++;
   }
   
   if (row<MAXNUMNEIGHBORS) {
      
      // add this neighbor
      neighbors_vars.neighbors[row].used                   = TRUE;
      neighbors_vars.neighbors[row].parentPreference       = 0;
      // neighbors_vars.neighbors[row].stableNeighbor         = FALSE;
      // Note: all new neighbors are consider stable
      neighbors_vars.neighbors[row].stableNeighbor         = TRUE;
      neighbors_vars.neighbors[row].switchStabilityCounter = 0;
      memcpy(&neighbors_vars.neighbors[row].addr_64b,address,sizeof(open_addr_t));
      neighbors_vars.neighbors[row].rank                   = DEFAULTDAGRANK;
      neighbors_vars.neighbors[row].rssi                   = rssi;
      neighbors_vars.neighbors[row].numRx                  = 1;
      neighbors_vars.neighbors[row].numTx                  = 0;
      neighbors_vars.neighbors[row].numTxACK               = 0;
      memcpy(&neighbors_vars.neighbors[row].asn,asnTimestamp,sizeof(asn_t));
      //update jp
      if (joinPrioPresent==TRUE){
         neighbors_vars.neighbors[row].joinPrio=joinPrio;
      }
      
      // do I already have a preferred parent ? -- TODO change to use JP
      iHaveAPreferedParent = FALSE;
      for (j=0;j<MAXNUMNEIGHBORS;j++) {
         if (neighbors_vars.neighbors[j].parentPreference==MAXPREFERENCE) {
            iHaveAPreferedParent = TRUE;
         }
      }
      // if I have none, and I'm not DAGroot, the new neighbor is my preferred
      if (iHaveAPreferedParent==FALSE && idmanager_getIsDAGroot()==FALSE) {      
         neighbors_vars.neighbors[row].parentPreference     = MAXPREFERENCE;
      }
   } else {
      openserial_printError(COMPONENT_NEIGHBORS,ERR_NEIGHBORS_FULL,
                            (errorparameter_t)MAXNUMNEIGHBORS,
                            (errorparameter_t)0);
   }
}

uint8_t findNeighborRow(open_addr_t* neighbor) {
   uint8_t row=0;
   while (row<MAXNUMNEIGHBORS) {
      if (isThisRowMatching(neighbor,row)) {
         break;
      }
      row++;
   }
   return row;
}

void removeNeighbor(uint8_t neighborIndex) {
   memset(&(neighbors_vars.neighbors[neighborIndex]),0,sizeof(neighborRow_t));
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

bool neighbors_isSameDODAGID() {
   return memcmp(
      &(neighbors_vars.myDODAGID[0]),
      &(neighbors_vars.dio->DODAGID[0]),
      sizeof(neighbors_vars.dio->DODAGID)
   );
}

bool neighbors_isDODAGverNumbHigher(DODAGversion_t firstVerNumb, DODAGversion_t secondVerNumb) {
   return (firstVerNumb > secondVerNumb);
}
