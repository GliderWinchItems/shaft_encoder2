/******************************************************************************
* File Name          : levelwind_idx_v_struct.h
* Date First Issued  : 09/23/2020
* Board              :
* Description        : Load parameter struct
*******************************************************************************/

#include <stdint.h>
#include "common_can.h"
#include "LevelwindTask.h"

#ifndef __LEVELWIND_IDX_V_STRUCT
#define __LEVELWIND_IDX_V_STRUCT

/* GevcuTask counts 'sw1timer' ticks for various timeouts.
 SWTIM1TICKDURATION
 We want the duration long, but good enough resolution(!)
 With systick at 512/sec, specifying 8 ms yields a 4 tick duration
 count = 4 -> 64/sec (if we want to approximate the logging rate)
 count = 64 -> 1/sec 
*/ 
#define SWTIM1TICKDURATION 8
#define SWTIM1TICKPERSEC (1000/SWTIM1TICKDURATION)

#define SWTIM1_64PERSEC (configTICK_RATE_HZ/64) // swtim1 ticks 

/* Parameters levelwind instance (LC = Local Copy) */
struct LEVELWINDLC
 {
/* NOTE: all suffix _t parameters are times in milliseconds */

	uint32_t size;
	uint32_t crc;   // TBD
   uint32_t version;   //

   // this belongs somewhere associated with the node, not the LW
   uint8_t  mydrum;     // the drum number for this node 

	/* Timings in milliseconds. Converted later to 'swtim1' ticks. */
	uint32_t hbct_t;     // Heartbeat ct: ms between sending 
   uint32_t hbct;       // Number of ticks between hb msgs

   int32_t  Lplus;      // start of positive reversal region 
   int32_t  Lminus;     // start of negative reversal region
   int32_t  Ka;         // reversal rate
   int32_t  Nr;         // ratio of reversal rate to sweep rate


   uint32_t ocidx;      // OC register increment for indexing 
   uint8_t  Nswp;       // sweep rate speed-up factor
   int32_t  Nman;       // manual rate reduction factor

   // For development, these will likely not be needed in operational code
   float    clfactor;   // Constant to compute oc duration at CL = 100.0
   uint32_t cltimemax;  // Max timer count for shutdown 
   uint32_t ka_levelwind_t; // keepalive from PC (ms)     

 // CAN ids ...........................................................................
   //                                  CANID_NAME             CAN_MSG_FMT     DESCRIPTION
    // Levelwind sends; PC receives
   uint32_t cid_hb_levelwind;        // CANID_HB_LEVELWIND: U8_U32','LEVELWIND: U8: Status, U32: stepper position accum


 // List of CAN ID's for setting up hw filter for incoming msgs
	// stepper test repo sends: drum receives
  uint32_t cid_gps_sync;         // CANID_HB_TIMESYNC   00400000  U8     GPS time sync distribution msg-GPS time sync msg
	uint32_t cid_drum_tst_stepcmd; // CANID_TST_STEPCMD   E4600000  U8_FF DRUM1: U8: Enable,Direction, FF: CL position: 
  uint32_t cid_mc_state;         // CANID_MC_STATE      26000000  MC', 'UNDEF','MC: Launch state msg');
  uint32_t cid_unit_encoder;     // E.G., CANID_UNIT_ENCODER1 83200000  U8_VAR DiscoveryF4 encoder demo winch
 };

/* *************************************************************************/
void levelwind_idx_v_struct_hardcode_params(struct LEVELWINDLC* p);
/* @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * *************************************************************************/
 
#endif

