/******************************************************************************
* File Name          : OdometerTask.c
* Date First Issued  : 12/12/2024
* Description        : Odometer function w STM32CubeMX w FreeRTOS
*******************************************************************************/

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "malloc.h"

#include "main.h"
#include "morse.h"
#include "drum_items.h"
#include "MailboxTask.h"
#include "controlpanel_items.h"
#include "OdometerTask.h"
#include "odometer_items.h"
#include "../../../GliderWinchCommons/embed/svn_common/trunk/db/gen_db.h"

uint32_t debugodo1;

/* Private functions and macros to the file */

osThreadId OdometerTaskHandle;

uint32_t dbgEth;

struct ODOMETERFUNCTION odometerfunction;
struct CONTROLPANELSTATE cp_state;

#if 1
/* *************************************************************************
 * static void toggle_led(void);
 * @brief   : Debugging 
 * *************************************************************************/
static uint32_t zledctr1;
static void toggle_led(void)
{
   zledctr1 += 1;
   if (zledctr1 >= 32)        
   {
      zledctr1 = 0;
      HAL_GPIO_TogglePin(GPIOD,LED_RED_Pin);
   }
}   
#endif

/* *************************************************************************
 * void swtim1_callback(TimerHandle_t tm);
 * @brief	: Software timer 1 timeout callback
 * *************************************************************************/
static void swtim1_callback(TimerHandle_t tm)
{
	xTaskNotify(OdometerTaskHandle, ODOMETERNOTBITSWT1, eSetBits);
	return;
}

/* *************************************************************************
 * void StartOdometerTask(void const * argument);
 *	@brief	: Task startup
 * *************************************************************************/
struct SWITCHPTR* psw_safeactivex; // Debugging

void StartOdometerTask(void const * argument)
{
	struct ODOMETERFUNCTION* p = &odometerfunction; // Convenience pointer
//   struct CONTROLPANELSTATE* pcp = &cp_state;   // Convenience pointer

	/* A notification copies the internal notification word to this. */
	uint32_t noteval = 0;    // Receives notification word upon an API notify

   // Load parameters
   odometer_idx_v_struct_hardcode_params(&p->lc);

   /* Add CAN Mailboxes                        CAN           CAN ID        Notify bit   Paytype */
   p->pmbx_cid_gps_sync    =  MailboxTask_add(pctl0,p->lc.cid_gps_sync,NULL,ODOMETERNOTBITSYNC,0,U8);
   p->pmbx_cid_mc_state    =  MailboxTask_add(pctl0,p->lc.cid_mc_state,NULL,ODOMETERNOTBITMC,0,U8);
   p->pmbx_cid_cmd_encoder =  MailboxTask_add(pctl0,p->lc.cid_cmd_encoder ,NULL,ODOMETERNOTBITCMD,0,U8);

extern void odometer_items_init(struct ODOMETERFUNCTION* p);
   odometer_items_init(p);

    /* Create timer #1: hearbeat (2 per sec) */
	odometerfunction.swtim1 = xTimerCreate("swtim1",
		   pdMS_TO_TICKS(100), 
		   pdTRUE, (void *) 0, 
		   swtim1_callback);
	if (odometerfunction.swtim1 == NULL) {morse_trap(404);}

	/* Start heart-beat timer 100 ms ticks */
	BaseType_t bret = xTimerReset(p->swtim1, pdMS_TO_TICKS(100));
	if (bret != pdPASS) {morse_trap(405);}

extern CAN_HandleTypeDef hcan1;
	HAL_CAN_Start(&hcan1); // CAN1

	for (;;)
	{
		/* Wait for notifications */
		xTaskNotifyWait(0,0xffffffff, &noteval, portMAX_DELAY);

      if ((noteval & ODOMETERNOTBITSWT1) != 0)
		{ // Software timer callback caused this
debugodo1 += 1;         
         if (p->hbctr > 0)
            p->hbctr -= 1;
		}

      if ((noteval & ODOMETERNOTBITSYNC) != 0)
      { // gps/logger time sync msg ('00400000')
      }

      if ((noteval & ODOMETERNOTBITMC) != 0)
      { // Master controller state msg ('26000000')
      }

      if ((noteval & ODOMETERNOTBITCMD) != 0)
      { // command/request ('83600000'|'83800000')
      }

      if ((noteval & ODOMETERNOTBITTIM2) != 0)
      { // New set of encoder readings
toggle_led();         
         odometer_items_compute();
         odometer_items_hearbeat();
      }
	}
}

/* *************************************************************************
 * osThreadId xOdometerTaskCreate(uint32_t taskpriority);
 * @brief	: Create task; task handle created is global for all to enjoy!
 * @param	: taskpriority = Task priority (just as it says!)
 * @return	: OdometerTaskHandle
 * *************************************************************************/
osThreadId xOdometerTaskCreate(uint32_t taskpriority)
{
 	osThreadDef(OdometerTask, StartOdometerTask, osPriorityNormal, 0, (192));
	OdometerTaskHandle = osThreadCreate(osThread(OdometerTask), NULL);
	vTaskPrioritySet( OdometerTaskHandle, taskpriority );
	return OdometerTaskHandle;
}
