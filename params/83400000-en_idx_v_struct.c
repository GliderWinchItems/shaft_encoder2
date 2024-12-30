/******************************************************************************
* File Name          : 83400000_odometer_idx_v_struct.c
* Date First Issued  : 12/10/2024
* Description        : Odometer function parameters for CAN ID XXXXXXXX
* Usage              : Winch demo proxy DiscoveryF4
*******************************************************************************/
#include "odometer_idx_v_struct.h"
#include "SerialTaskReceive.h"
#include "../../../GliderWinchCommons/embed/svn_common/trunk/db/gen_db.h"

/* *************************************************************************
 * void odometer_idx_v_struct_hardcode_params(truct ODOMETERLC* p);
 * @brief	: Init struct from hard-coded parameters (rather than database params in highflash)
 * @return	: 0
 * *************************************************************************/
void odometer_idx_v_struct_hardcode_params(struct ODOMETERLC* p)
{
/* GevcuTask counts 'sw1timer' ticks for various timeouts.
 We want the duration long, but good enough resolution(!)
 With systick at 512/sec, specifying 8 ms yields a 4 tick duration
 count = 4 -> 64/sec (if we want to approximate the logging rate)
 count = 64 -> 1/sec 
*/ 
//    my drum should be associated with whole node and not just the
//    this function. need to figure out where the parameters for 
//    the whole node should be placed       
   p->mydrum  =    1; // drum this node is assigned to     
   p->hb_t    = 4000; // Heartbeat: milliseconds between sending 
   p->zspd_t  =  500; // Zero drum speed detection duration (ms)
   
   p->rim_to_rope_default =   150f; // Distance (mm), drum rim -to- loaded rope
   p->rope_total_default  =  1000f; // Total rope loaded on drum (meters)
   p->hub_bare_dia    = 0.750f; // Drum without rope diameter (meters)
   p->rope_dia        = 4.5f;   // Effective dia for drum loading computations (mm)
   p->drum_outer_dia  = 0.975f; // Drum flange outer diameter(meters)
   p->drum_width      = 0.165f; // Drum inside width (meters)
   p->rim_to_cushion = (p->drum_outer_dia - p->hub_bare_dia)*0.5f; // Temporary
   p->encoder_ratio   =  6.44f; // Gear ratio--motor:drum
   p->scale_en_mtr    =  840.0E3f; // encoder ct/time scale to motor rpm
   p->scale_mtr_drum  = 0.155279503f;// motor rpm scale to drum rpm
   p->scale_en_circum = 737.96964f; // encoder to drum circumference (meters)
   p->scale_speediff_accel = 1.0E0f; // speed diff sum to motor accel (rad/sec/sec)


// CAN ids encoder function
   //                         CANID_NAME      CANID_HEX  CAN_MSG_FMT     DESCRIPTION
   // We send; Others receive
   p->cid_unit_encoder  = CANID_UNIT_ENCODER2; // 83400000 U8_VAR DiscoveryF4 encoder proxy
   p->cid_msg1_encoder  = CANID_MSG1_ENCODER2; // 83C00000 FF_FF  DiscoveryF4 encoder proxy: lineout, speed
   p->cid_msg2_encoder  = CANID_MSG2_ENCODER2; // 84000000 FF_FF  DiscoveryF4 encoder proxy: accel, encoder ctr

   // We receive
   p->cid_gps_sync     = CANID_HB_TIMESYNC;  // 00400000 U8     GPS time sync distribution msg-GPS time sync msg
   p->cid_mc_state     = CANID_MC_STATE;     // 26000000 MC     MC Launch state msg
   p->cid_cmd_encoder  = CANID_CMD_ENCODER2; // 83800000 U8_VAR DiscoveryF4 encoder proxy: command

	return;
}
