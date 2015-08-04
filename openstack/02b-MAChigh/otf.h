#ifndef __OTF_H
#define __OTF_H

/**
\addtogroup MAChigh
\{
\addtogroup otf
\{
*/

#include "opendefs.h"

//=========================== define ==========================================

//=========================== typedef =========================================

typedef enum {
   OTF_IDLE                                     = 0x00,   // ready for next event
   // states set by housekeeping 
   OTF_SENDCOMMAND_ADD                          = 0x01,   // waiting for SendDone confirmation
   OTF_SENDCOMMAND_REMOVE                       = 0x02,   // waiting for response from the neighbor
   // state set after a command has been sent to sixtop
   OTF_WAIT_SIXTOP_ADD                          = 0x03,
   OTF_WAIT_SIXTOP_REMOVE                       = 0x04,
   // state set if cells have been added
   OTF_RESET                                    = 0x05
} otf_state_t;

typedef struct {
   otf_state_t          state;
   uint8_t              requiredVirtualCells;
   uint8_t              aveRequiredVirtualCells;
   uint8_t              scheduledVirtualCells;
   uint16_t             numCells;
   open_addr_t          currentNeighbor;
} otf_vars_t;

//=========================== module variables ================================

//=========================== prototypes ======================================

// admin
void      otf_init(void);
// notification from sixtop
void      otf_housekeeping(void);
void      otf_notif_release(owerror_t error);

/**
\}
\}
*/

#endif
