#include "opendefs.h"
#include "otf.h"
#include "neighbors.h"
#include "schedule.h"
#include "sixtop.h"
#include "scheduler.h"

//=========================== variables =======================================

otf_vars_t otf_vars;

//=========================== prototypes ======================================

void otf_addORremoveCell_task(void);
void otf_reset(void);

//=========================== public ==========================================

void otf_init(void) {
   otf_reset();
}

void otf_housekeeping() {
   uint8_t        neighborIndex;
   open_addr_t    address;
   bool           f_shouldRemove;
   otf_vars_t     otf_vars_remove;
   
   if (otf_vars.state == OTF_IDLE) {
      memset(&address,0,sizeof(open_addr_t));
      f_shouldRemove = FALSE;
      for (neighborIndex=0;neighborIndex<MAXNUMNEIGHBORS;neighborIndex++) {
         if (neighbors_getOTFstatisticsByIndex(
               neighborIndex,
               &address,
               &(otf_vars.requiredVirtualCells),
               &(otf_vars.aveRequiredVirtualCells)
            )==TRUE) {
            otf_vars.scheduledVirtualCells = schedule_getVirtualScheduledCellsToNeighbor(&address);
            memcpy(&(otf_vars.currentNeighbor),&address,sizeof(open_addr_t));
            if (otf_vars.requiredVirtualCells > otf_vars.scheduledVirtualCells) {
               otf_vars.numCells = otf_vars.requiredVirtualCells - otf_vars.scheduledVirtualCells;
               otf_vars.state = OTF_SENDCOMMAND_ADD;
               break;
            } else if (
                  (otf_vars.scheduledVirtualCells < 2 * otf_vars.aveRequiredVirtualCells)
                  &&
                  (otf_vars.requiredVirtualCells < 2 * otf_vars.aveRequiredVirtualCells - otf_vars.scheduledVirtualCells)
                  &&
                  (f_shouldRemove == FALSE)
               ) {
               otf_vars.numCells = otf_vars.aveRequiredVirtualCells - otf_vars.requiredVirtualCells;
               memcpy(&otf_vars_remove,&otf_vars,sizeof(otf_vars_t));
               f_shouldRemove = TRUE;
            }
         }
      }
      if (otf_vars.state != OTF_IDLE) {
         scheduler_push_task(otf_addORremoveCell_task,TASKPRIO_OTF);
      } else if (f_shouldRemove == TRUE) {
         memcpy(&otf_vars,&otf_vars_remove,sizeof(otf_vars_t));
         otf_vars.state = OTF_SENDCOMMAND_REMOVE;
         scheduler_push_task(otf_addORremoveCell_task,TASKPRIO_OTF);
      } else {
         otf_reset();
      }
   } else if (otf_vars.state == OTF_RESET) {
      otf_reset();
   }
}

void otf_notif_release(owerror_t error) {
   /*printf("%x->%x, OTF , REQ %d, MINREQ %d, SCH %d, ",
      idmanager_getMyID(ADDR_64B)->addr_64b[7],
      otf_vars.currentNeighbor.addr_64b[7],
      otf_vars.requiredVirtualCells,
      otf_vars.aveRequiredVirtualCells,
      otf_vars.scheduledVirtualCells);
   if (otf_vars.state == OTF_WAIT_SIXTOP_ADD) {
      printf("ADDED ");
      if (error == E_SUCCESS) {
         printf("%d CELLS",otf_vars.numCells);
      } else {
         printf("FAILED");
      }
   } else if (otf_vars.state == OTF_WAIT_SIXTOP_REMOVE) {
      printf("REMOVED ");
      if (error == E_SUCCESS) {
         printf("%d CELLS",otf_vars.numCells);
      } else {
         printf("FAILED");
      }
   } else {
      printf("RELEASE SHOULD NOT HAPPEN");
   }
   printf("\n");*/
   otf_vars.state = OTF_RESET;
}

//=========================== private =========================================

void otf_addORremoveCell_task(void) {
   owerror_t outcome;
   
   // call sixtop
   if (otf_vars.state == OTF_SENDCOMMAND_ADD) {
      outcome = sixtop_addCells(&(otf_vars.currentNeighbor),otf_vars.numCells,SIX_HANDLER_OTF);
   } else if (otf_vars.state == OTF_SENDCOMMAND_REMOVE) {
      outcome = sixtop_removeCells(&(otf_vars.currentNeighbor),otf_vars.numCells,SIX_HANDLER_OTF);
   } else {
      outcome = E_FAIL;
   }
   
   if (outcome == E_SUCCESS) {
      if (otf_vars.state == OTF_SENDCOMMAND_ADD) {
         otf_vars.state = OTF_WAIT_SIXTOP_ADD;
      } else if (otf_vars.state == OTF_SENDCOMMAND_REMOVE) {
         otf_vars.state = OTF_WAIT_SIXTOP_REMOVE;
      }
   } else {
      otf_vars.state = OTF_RESET;
   }
}

void otf_reset() {
   memset(&otf_vars,0,sizeof(otf_vars_t));
   otf_vars.state = OTF_IDLE;
   neighbors_resetOTFstatistics();
}
