/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "SerialTaskSend.h"
#include "stm32f4xx_hal_pcd.h"
#include "CanTask.h"
#include "can_iface.h"
#include "canfilter_setup.h"
#include "stm32f4xx_hal_can.h"
#include "getserialbuf.h"
#include "stackwatermark.h"
#include "yprintf.h"
#include "DTW_counter.h"
#include "SerialTaskReceive.h"
#include "yscanf.h"
#include "morse.h"
#include "MailboxTask.h"
#include "OdometerTask.h"
#include "odometer_items.h"
#include "levelwind_items.h"
#include "drum_items.h"
#include "LevelwindTask.h"
#include "levelwind_switches.h"
//#include "DrumTask.h"
//#include "BrakeTask.h"
#include "ADCTask.h"
#include "adcparams.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

void* verr[8];
uint32_t verrx = 0;
__attribute__( ( always_inline ) ) __STATIC_INLINE uint32_t __get_SP(void) 
{ 
  register uint32_t result; 

  __ASM volatile ("MOV %0, SP\n" : "=r" (result) ); 
  return(result); 
} 

uint32_t timectr = 0;
struct CAN_CTLBLOCK* pctl0;	// Pointer to CAN1 control block
struct CAN_CTLBLOCK* pctl1;	// Pointer to CAN2 control block

uint32_t debugTX1b;
uint32_t debugTX1b_prev;

uint32_t debugTX1c;
uint32_t debugTX1c_prev;

uint32_t debug03;
uint32_t debug03_prev;

extern osThreadId SerialTaskHandle;
extern osThreadId CanTxTaskHandle;
extern osThreadId CanRxTaskHandle;
extern osThreadId SerialTaskReceiveHandle;

uint16_t m_trap = 450; // Trap codes for MX Init() and Error Handler

uint8_t canflag;
uint8_t canflag1;
uint8_t canflag2;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

osThreadId defaultTaskHandle;
osTimerId defaultTaskTimerHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);
void CallbackdefaultTaskTimer(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	BaseType_t ret;	   // Used for returns from function calls
	osMessageQId Qidret; // Function call return
	osThreadId Thrdret;  // Return from thread create

// Debug: Clear heap area
//uint32_t* pclr = (uint32_t*)(0x2000bb80);
//while (pclr < (uint32_t*)(0x2000bb80 + 32768)) *pclr++ = 0x66666666;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	DTW_counter_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
	/* Add bcb circular buffer to SerialTaskSend for usart3 -- PC monitor */
	#define NUMCIRBCB3  16 // Size of circular buffer of BCB for usart3
	ret = xSerialTaskSendAdd(&HUARTMON, NUMCIRBCB3, 1); // dma
	if (ret < 0) morse_trap(14); // Panic LED flashing

	/* Setup TX linked list for CAN  */
   // CAN1 (CAN_HandleTypeDef *phcan, uint8_t canidx, uint16_t numtx, uint16_t numrx);
	pctl0 = can_iface_init(&hcan1, 0, 32, 32);
	if (pctl0 == NULL) morse_trap(7); // Panic LED flashing
	if (pctl0->ret < 0) morse_trap(77);

	// CAN 2
#ifdef CONFIGCAN2
	pctl1 = can_iface_init(&hcan2, 1,32, 64);
	if (pctl1 == NULL) morse_trap(8); // Panic LED flashing
	if (pctl1->ret < 0) morse_trap(88);
#endif

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of defaultTaskTimer */
  osTimerDef(defaultTaskTimer, CallbackdefaultTaskTimer);
  defaultTaskTimerHandle = osTimerCreate(osTimer(defaultTaskTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

	/* defaultTask timer for pacing defaultTask output. */
//	ret = xTimerChangePeriod( defaultTaskTimerHandle  ,pdMS_TO_TICKS(64),0);

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 384);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
/* =================================================== */
  /* init code for USB_DEVICE */

//taskENTER_CRITICAL();
  //MX_USB_DEVICE_Init();
  //osDelay(0);
//taskEXIT_CRITICAL();


	/* Create serial task (priority) */
	// Task handle "osThreadId SerialTaskHandle" is global
	Thrdret = xSerialTaskSendCreate(1);	// Create task and set Task priority
	if (Thrdret == NULL) morse_trap(225);

	/* Create serial receiving task. */
//	ret = xSerialTaskReceiveCreate(1);
//	if (ret != pdPASS) morse_trap(224);

	/* Setup semaphore for yprint and sprintf et al. */
	yprintf_init();

  /* definition and creation of CanTxTask - CAN driver TX interface. */
  Qidret = xCanTxTaskCreate(1, 64); // CanTask priority, Number of msgs in queue
	if (Qidret < 0) morse_trap(220); // Panic LED flashing

  /* definition and creation of CanRxTask - CAN driver RX interface. */
  /* The MailboxTask takes care of the CANRx                         */
//  Qidret = xCanRxTaskCreate(1, 32); // CanTask priority, Number of msgs in queue
//	if (Qidret < 0) morse_trap(6); // Panic LED flashing

	/* Setup CAN hardware filters to default to accept all ids. */
	HAL_StatusTypeDef Cret;
	Cret = canfilter_setup_first(0, &hcan1, 15); // CAN1
	if (Cret == HAL_ERROR) morse_trap(219);

#ifdef CONFIGCAN2
	Cret = canfilter_setup_first(0, &hcan2, 15); // CAN2
	if (Cret == HAL_ERROR) morse_trap(217);
#endif

	/* Remove "accept all" CAN msgs and add specific id & mask, or id here. */
	// See canfilter_setup.h

	/* Create MailboxTask */
	xMailboxTaskCreate(osPriorityNormal+1); // (arg) = priority

	/* Create Mailbox control block w 'take' pointer for each CAN module. */
	struct MAILBOXCANNUM* pmbxret;
	// (CAN1 control block pointer, size of circular buffer)
	pmbxret = MailboxTask_add_CANlist(pctl0, 48);
	if (pmbxret == NULL) morse_trap(215);

#ifdef CONFIGCAN2
	// (CAN2 control block pointer, size of circular buffer)
	pmbxret = MailboxTask_add_CANlist(pctl1, 48); // Use default buff size
	if (pmbxret == NULL) morse_trap(214);
#endif

  /* ADC summing, calibration, etc. */
  xADCTaskCreate(1); // (arg) = priority

  /* Odometer task */
  Thrdret = xOdometerTaskCreate(osPriorityNormal+2); // (arg) = priority
  if (Thrdret == NULL) morse_trap(2164); 

  /* Levelwind (stepper) task */
//  Thrdret = xLevelwindTaskCreate(5); // (arg) = priority
//  if (Thrdret == NULL) morse_trap(2161); 

  /* Drum task */
//  Thrdret = xDrumTaskCreate(4); // (arg) = priority
//  if (Thrdret == NULL) morse_trap(2162); 

  /* Brake task */
//  Thrdret = xBrakeTaskCreate(1); // (arg) = priority
//  if (Thrdret == NULL) morse_trap(2163); 

	/* Further initialization of mailboxes takes place when tasks start */

	/* Select interrupts for CAN1 */
	HAL_CAN_ActivateNotification(&hcan1, \
		CAN_IT_TX_MAILBOX_EMPTY     |  \
		CAN_IT_RX_FIFO0_MSG_PENDING |  \
		CAN_IT_RX_FIFO1_MSG_PENDING    );

	/* Select interrupts for CAN2 */
#ifdef CONFIGCAN2
	HAL_CAN_ActivateNotification(&hcan2, \
		CAN_IT_TX_MAILBOX_EMPTY     |  \
		CAN_IT_RX_FIFO0_MSG_PENDING |  \
		CAN_IT_RX_FIFO1_MSG_PENDING    );
#endif

/* =================================================== */

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
	m_trap = 456; // morse_trap(456);
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */
	m_trap = 451; // morse_trap(451);
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_GREEN_Pin LED_ORANGE_Pin LED_RED_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
// ################################################################################
// ######### DEFAULT TASK #########################################################
// ################################################################################
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

osDelay(0); // Debugging HardFault

/* Select code for testing/monitoring by uncommenting #defines */
//#define DISPLAYSTACKUSAGEFORTASKS
//#define ADCSHOW
//#define SHOWADCCOMMONCOMPUTATIONS
//#define STEPPERSHOW
//#define ENCODERSHOW
#define SHOWENTIMCT

	#define DEFAULTTSKBIT00	(1 << 0)  // Task notification bit for sw timer: stackusage
	#define DEFAULTTSKBIT01	(1 << 1)  // Task notification bit for sw timer: something else

	/* A notification copies the internal notification word to this. */
	uint32_t noteval = 0;    // Receives notification word upon an API notify

	struct SERIALSENDTASKBCB* pbuf1 = getserialbuf(&HUARTMON,96);
	if (pbuf1 == NULL) morse_trap(11);

	struct SERIALSENDTASKBCB* pbuf3 = getserialbuf(&HUARTMON,96);
	if (pbuf3 == NULL) morse_trap(13);

	struct SERIALSENDTASKBCB* pbuf2 = getserialbuf(&HUARTMON,96);
	if (pbuf2 == NULL) morse_trap(12);

	struct SERIALSENDTASKBCB* pbuf4 = getserialbuf(&HUARTMON,96);	
	if (pbuf4 == NULL) morse_trap(12);

yprintf(&pbuf4,"\n\rshaft_encoder2 starts\n\r");
  
#ifdef DISPLAYSTACKUSAGEFORTASKS
	int ctr = 0; // Running count
	uint32_t heapsize;
	uint32_t showctr = 0;
	uint32_t t1_DSUFT; // DTW time  
	uint32_t t2_DSUFT;
#endif


HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15); // BLUE LED


#ifdef  SHOWSERIALPARALLELSTUFF
uint32_t spispctr_prev = 0;
#endif

/* Countdown timer for various display speeds. */
uint16_t slowtimectr = 0; // Approx 1/sec
uint16_t medtimectr = 0;  // Approx 8/sec

uint8_t ratepace = 0;

//osDelay(1);
	xTimerChangePeriod( defaultTaskTimerHandle  ,pdMS_TO_TICKS(16),0);

#ifdef ADCSHOW
//  extern uint32_t adcsumdb[6]; // debug
  extern uint16_t* p16;
  uint32_t ctr1_prev = 0;
  uint32_t ct = 0;
#endif  

#ifdef STEPPERSHOW
  uint32_t stepctr = 1;
#endif  
// ===== BEGIN FOR LOOP ==============================

	for ( ;; )
	{
		xTaskNotifyWait(0,0xffffffff, &noteval, portMAX_DELAY);
    if ((noteval & DEFAULTTSKBIT01) != 0)
    {

#ifdef SHOWENTIMCT      
      struct ODOMETERFUNCTION* pe = &odometerfunction; // Convenience pointer

      yprintf(&pbuf1,"\n\r%7d %4d", pe->odotimct_int_diff[0].tim, pe->odotimct_int_diff[0].ct);
      yprintf(&pbuf2," %7d %4d",    pe->odotimct_int_diff[1].tim, pe->odotimct_int_diff[1].ct);
      yprintf(&pbuf3," %7d %4d",    pe->odotimct_int_diff[2].tim, pe->odotimct_int_diff[2].ct);
      yprintf(&pbuf4," %7d %4d %4d",pe->odotimct_int_diff[3].tim, pe->odotimct_int_diff[3].ct, pe->en_cnt_diff);
      yprintf(&pbuf1," %9.3f", pe->odo_speed_ave_motor);
      yprintf(&pbuf2," %9.3f", pe->accel_ave_motor);
      yprintf(&pbuf3," %9.3f", pe->en_cnt_speed);
      yprintf(&pbuf4," %9.3f", pe->en_cnt_accel_motor);

#endif      
      }

		if ((noteval & DEFAULTTSKBIT00) != 0)
		{
// ================= Higest rate =======================================
    ratepace += 1;
    if (ratepace > 0) // Slow down LCD output rate if desired
    {
      ratepace = 0; 
    }
// ================== SLOW ==============================================
/* Countdown timer notifications. */
			slowtimectr += 1;
			if (slowtimectr >= 64)
			{
				slowtimectr = 0;

#ifdef ADCSHOW

  yprintf(&pbuf4,"\n\rA%4i %9i %6i %6i %6i %6i %6i",ct, adc1.ctr-ctr1_prev, 
    *(p16+0),*(p16+1),*(p16+2),*(p16+3),*(p16+4) );

    yprintf(&pbuf4,"\n\rB%4i %9i %6i %6i %6i %6i %6i",ct++, adc1.ctr-ctr1_prev, 
    *(p16+5),*(p16+6),*(p16+7),*(p16+8),*(p16+9) );

//    adcsumdb[0],adcsumdb[1],adcsumdb[2],adcsumdb[3],adcsumdb[4]);
  ctr1_prev = adc1.ctr;
#endif      

#ifdef ENCODERSHOW
  struct ODOMETERFUNCTION* pen = &odometerfunction; // Convenience pointer

  yprintf(&pbuf1,"\n\rspeed_sum%14.8f",pen->speed_sum);
  
  float ftmp1 = pen->speed_sum * pen->lc.scale_en_mtr; // Scale raw->motor rpm
  yprintf(&pbuf4,"\n\r_ave_mtr %12.6f %14.6f",ftmp1, pen->en_cnt_speed);

  ftmp1 = ftmp1 * pen->lc.scale_mtr_drum;  // Scale motor rpm->drum rpm
  yprintf(&pbuf4,"\n\r_ave_drum%12.6f",ftmp1);

  yprintf(&pbuf2,"\n\r    tim :%12d%12d%12d%12d"    ,pen->odotimct_buff[0].tim,pen->odotimct_buff[1].tim,pen->odotimct_buff[2].tim,pen->odotimct_buff[3].tim);
  yprintf(&pbuf3,"\n\r    ct  :%12d%12d%12d%12d"    ,pen->odotimct_buff[0].ct ,pen->odotimct_buff[1].ct ,pen->odotimct_buff[2].ct ,pen->odotimct_buff[3].ct );
  yprintf(&pbuf3,"\n\rdiff.tim:%12d%12d%12d%12d"    ,pen->odotimct_int_diff[0].tim,pen->odotimct_int_diff[1].tim,pen->odotimct_int_diff[2].tim,pen->odotimct_int_diff[3].tim);
  
  yprintf(&pbuf4,"\n\rdiff.ct :%12d%12d%12d%12d",pen->odotimct_int_diff[0].ct ,pen->odotimct_int_diff[1].ct ,pen->odotimct_int_diff[2].ct ,pen->odotimct_int_diff[3].ct );
  yprintf(&pbuf1,"%12d\n\r",pen->en_cnt_diff);

extern uint32_t debugodo1;
yprintf(&pbuf1," x %d\n\r",debugodo1);
#endif  


#ifdef DISPLAYSTACKUSAGEFORTASKS
			/* Display the amount of unused stack space for tasks. */
t1_DSUFT = DTWTIME;
			showctr += 1; 
/* 'for' is to test doing all scans at one timer tick. */
for (showctr = 0; showctr < 7; showctr++)
{
				switch (showctr)
				{
/* Cycle through the tasks. */
case  0: stackwatermark_show(defaultTaskHandle,&pbuf1,"defaultTask--");break;
case  1: stackwatermark_show(SerialTaskHandle ,&pbuf2,"SerialTask---");break;
case  2: stackwatermark_show(CanTxTaskHandle  ,&pbuf3,"CanTxTask----");break;
case  3: stackwatermark_show(MailboxTaskHandle,&pbuf4,"MailboxTask--");break;
case  4: stackwatermark_show(SerialTaskReceiveHandle,&pbuf1,"SerialRcvTask");break;
case  5: stackwatermark_show(ADCTaskHandle,    &pbuf4,"ADCTask------");break;

case 6:	heapsize = xPortGetFreeHeapSize(); // Heap usage (and test fp working.
			yprintf(&pbuf1,"\n\rGetFreeHeapSize: total: %i free %i %3.1f%% used: %i",configTOTAL_HEAP_SIZE, heapsize,\
				100.0*(float)heapsize/configTOTAL_HEAP_SIZE,(configTOTAL_HEAP_SIZE-heapsize)); break;
default: showctr=0; yprintf(&pbuf1,"\n\r%4i Unused Task stack space--", ctr++); break;
				}
}
t2_DSUFT = DTWTIME;
yprintf(&pbuf2,"\n\rDTW DUR: %d",t2_DSUFT - t1_DSUFT);

#endif

#ifdef SHOWADCCOMMONCOMPUTATIONS
      yprintf(&pbuf4,"\n\rADC: Vdd: %7.4f %8.4f   Temp: %6.1f %6.1f",adcommon.fvdd,adcommon.fvddfilt,adcommon.degC,adcommon.degCfilt);
      yprintf(&pbuf3,"\n\rInternal ref:   %d %d",adc1.chan[ADC1IDX_INTERNALVREF].sum/ADC1DMANUMSEQ, adcommon.ivdd);
      yprintf(&pbuf2,"\n\rCalibrated: vref: %6.4f",adc1.common.fvref);
#endif

  			}
// ================= Moderate ===================================
/* Countdown timer notifications. */
			medtimectr += 1;
			if (medtimectr >= 8)
			{
				medtimectr = 0;
    		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15); // BLUE LED
			}	
	  }
	}
  /* USER CODE END 5 */
}

/* CallbackdefaultTaskTimer function */
void CallbackdefaultTaskTimer(void const * argument)
{
  /* USER CODE BEGIN CallbackdefaultTaskTimer */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (defaultTaskHandle != NULL)
	{
		xTaskNotifyFromISR(defaultTaskHandle, 
			DEFAULTTSKBIT00,	/* 'or' bit assigned to buffer to notification value. */
			eSetBits,      /* Set 'or' option */
			&xHigherPriorityTaskWoken ); 
	}

  /* USER CODE END CallbackdefaultTaskTimer */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM12 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM12) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	morse_trap(m_trap); // Flash error code set in earlier MX Init()'s

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
morse_trap(222);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
