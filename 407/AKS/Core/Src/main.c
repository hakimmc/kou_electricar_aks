/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*extern uint8_t wav_2[33487];
extern uint8_t rawData2[627218];*/
//extern const uint8_t m[661848];
#include "electricar.h"

/* CAN Includes*/

	CAN_TxHeaderTypeDef pTxHeader;
	CAN_RxHeaderTypeDef pRxHeader;
	CAN_FilterTypeDef sFilterConfig;
	uint32_t pTxMailbox;

	uint8_t RxData[8];

	float converted_float[4];
	uint8_t data_bms[8];
	uint8_t data2_bms[8];
	uint8_t data3_bms[8];
	uint8_t data_motor;
	uint8_t data_charger;
	uint8_t data_relay;
	uint8_t CanReceive=0;
	uint8_t RxData_r[1];

	  char arr[100];
/* CAN Includes*/


int BMS_IDS[4] = {1,2,3,4};
int number=10;
int status=0;
int cnt = 0;
char tempp[24][4];
uint8_t adc_value;

char myMessagex[50];

struct VARIABLES{

		/*   veri sırası-----|
							 |
							 v

		 	 	 	 	 	 	 	 */
	/*          RF Values Start	     */
	int direction;
	int speed_for_rf;
	float battery_voltage[24];
	int battery_temparature;
	/*          RF Values End	     */
	int brake;
	float wheel_radius; // 0.5 meter


	int ADC_AVAILABLE;
	int ADC_CALCULATING;
	int CAN_FLAG;
	int RIGHT_SIGNAL_FLAG;
	int LEFT_SIGNAL_FLAG;
	int STOP_SIGNAL_FLAG;
	int ADC_FLAG;
	int VAR_FLAG;
}ecar;
#define power "t2"
#define speed "t12"
#define temp "t5"
#define watt "t10"
#define charge_percentage "t1"
#define clock "t0"
#define state_of_charge1 "p15"
#define state_of_charge2 "t7"
#define charge_time "t9"
#define state_of_charge3 "t11"
#define left_arrow "p6"
#define right_arrow "p9"
#define horn "p5"
#define lights "p13"
#define wiper "p11"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 CAN_HandleTypeDef hcan1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* Definitions for nextion */
osThreadId_t nextionHandle;
const osThreadAttr_t nextion_attributes = {
  .name = "nextion",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rf */
osThreadId_t rfHandle;
const osThreadAttr_t rf_attributes = {
  .name = "rf",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for can */
osThreadId_t canHandle;
const osThreadAttr_t can_attributes = {
  .name = "can",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for right_signal */
osThreadId_t right_signalHandle;
const osThreadAttr_t right_signal_attributes = {
  .name = "right_signal",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for left_signal */
osThreadId_t left_signalHandle;
const osThreadAttr_t left_signal_attributes = {
  .name = "left_signal",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for stop_signal */
osThreadId_t stop_signalHandle;
const osThreadAttr_t stop_signal_attributes = {
  .name = "stop_signal",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_DAC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
void start_nextion(void *argument);
void start_rf(void *argument);
void start_can(void *argument);
void right_signal_start(void *argument);
void left_signal_signal(void *argument);
void stop_signal_start(void *argument);


/* USER CODE BEGIN PFP */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void Set_LED (int LEDnum, int Red, int Green, int Blue);
void Set_Brightness (int brightness);
void WS2812_Send (void);
void Reset_LED (void);
void right_signal();
void stop_signal();
void left_signal();
float convert_float (int data_1, int data_2);
int data_merge();
void toggle_func(GPIO_TypeDef * port,uint16_t pin,int togg_flag,const int const_val);
int prepare_data();
void deneme();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MAX_LED 45
#define USE_BRIGHTNESS 0
#define l 15
#define RIGHT_SIGNAL MAX_LED-l
#define LEFT_SIGNAL MAX_LED-l-l-l
#define STOP_SIGNAL MAX_LED-l-l
#define PI 3.14159265


uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness

int datasentflag=0;



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_DAC_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_CAN_Start(&hcan1);


  pTxHeader.DLC = 1;
  pTxHeader.IDE = CAN_ID_STD;
  pTxHeader.RTR = CAN_RTR_DATA;
  pTxHeader.StdId = 0x0407;


  Reset_LED();

  Set_Brightness(45);
  WS2812_Send();
   //set filter parameters
   sFilterConfig.FilterActivation = ENABLE;
   sFilterConfig.FilterBank = 0;
   sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
   sFilterConfig.FilterIdHigh = 0x0000 ;
   sFilterConfig.FilterIdLow = 0x0000 ;
   sFilterConfig.FilterMaskIdHigh = 0x0000 ;
   sFilterConfig.FilterMaskIdLow = 0x0000 ;
   sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
   sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

   HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of nextion */
  nextionHandle = osThreadNew(start_nextion, NULL, &nextion_attributes);

  /* creation of rf */
  rfHandle = osThreadNew(start_rf, NULL, &rf_attributes);

  /* creation of can */
  canHandle = osThreadNew(start_can, NULL, &can_attributes);

  /* creation of right_signal */
  right_signalHandle = osThreadNew(right_signal_start, NULL, &right_signal_attributes);

  /* creation of left_signal */
  left_signalHandle = osThreadNew(left_signal_signal, NULL, &left_signal_attributes);

  /* creation of stop_signal */
  stop_signalHandle = osThreadNew(stop_signal_start, NULL, &stop_signal_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
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
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 105-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 3000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_start_nextion */
/**
  * @brief  Function implementing the nextion thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_start_nextion */
void start_nextion(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	  	 number++;
	  	 if(number>100){
	  		 number=10;
	  	 }
	  	 	 uint8_t cmdEnd[3] = { 0xFF, 0xFF, 0xFF };
	  	 	 uint8_t myMessage[50];
	  	 	 int lenx = sprintf(myMessage, "%s.txt=\"", "t12");
	  	 	 HAL_UART_Transmit(&huart6, (uint8_t*)myMessage , lenx, 1000);
	  	 	 HAL_UART_Transmit(&huart6, (uint8_t*)"20\"" , strlen("20\""), 1000);
	  	 	 HAL_UART_Transmit(&huart6, cmdEnd, 3, 1000);
	  	status= !status;
	  	 NEXTION_SEND(huart6,2, power, number, myMessagex);
	  	 NEXTION_SEND(huart6, 2, speed, number, myMessagex);
	  	 NEXTION_SEND(huart6, 2, temp, number, myMessagex);
	  	 NEXTION_SEND(huart6, 2, watt, number, myMessagex);
	  	 NEXTION_SEND(huart6, 2, charge_percentage, number, myMessagex);
	  	 NEXTION_SEND(huart6, 2, clock, number, myMessagex);
	  	 NEXTION_SEND(huart6, 0, state_of_charge1, status, myMessagex);
	  	 NEXTION_SEND(huart6, 0, state_of_charge2, status, myMessagex);
	  	 NEXTION_SEND(huart6, 2, charge_time, number, myMessagex);
	  	 NEXTION_SEND(huart6, 0, state_of_charge3, status, myMessagex);
	  	 NEXTION_SEND(huart6, 0, left_arrow, status, myMessagex);
	  	 NEXTION_SEND(huart6, 0, right_arrow, status, myMessagex);
	  	 NEXTION_SEND(huart6, 0, horn, status, myMessagex);
	  	 NEXTION_SEND(huart6, 0, lights, status, myMessagex);
	  	 NEXTION_SEND(huart6, 0, wiper, status, myMessagex);
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	  //HAL_Delay(20);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_start_rf */
/**
* @brief Function implementing the rf thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_rf */
void start_rf(void *argument)
{
  /* USER CODE BEGIN start_rf */
  /* Infinite loop */
ecar.direction='+';
  for(;;)
  {

	//sprintf(arr,"%c%c%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%c1",ecar.direction,ci2a(ecar.speed_for_rf),cf2a(ecar.battery_voltage[0],tempp[0]),cf2a(ecar.battery_voltage[1],tempp[1]),cf2a(ecar.battery_voltage[2],tempp[2]),cf2a(ecar.battery_voltage[3],tempp[3]),cf2a(ecar.battery_voltage[4],tempp[4]),cf2a(ecar.battery_voltage[5],tempp[5]),cf2a(ecar.battery_voltage[6],tempp[6]),cf2a(ecar.battery_voltage[7],tempp[7]),cf2a(ecar.battery_voltage[8],tempp[8]),cf2a(ecar.battery_voltage[9],tempp[9]),cf2a(ecar.battery_voltage[10],tempp[10]),cf2a(ecar.battery_voltage[11],tempp[11]),cf2a(ecar.battery_voltage[12],tempp[12]),cf2a(ecar.battery_voltage[13],tempp[13]),cf2a(ecar.battery_voltage[14],tempp[14]),cf2a(ecar.battery_voltage[15],tempp[15]),cf2a(ecar.battery_voltage[16],tempp[16]),cf2a(ecar.battery_voltage[17],tempp[17]),cf2a(ecar.battery_voltage[18],tempp[18]),cf2a(ecar.battery_voltage[19],tempp[19]),cf2a(ecar.battery_voltage[20],tempp[20]),cf2a(ecar.battery_voltage[21],tempp[21]),cf2a(ecar.battery_voltage[22],tempp[22]),cf2a(ecar.battery_voltage[23],tempp[23]),ci2a(ecar.battery_temparature));
	// deneme();
	  if(prepare_data()){
		 HAL_UART_Transmit(&huart2, (uint8_t*)arr, strlen(arr), 1000);
		 memset(arr, 0, 100 * sizeof(char));
		 HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		 HAL_Delay(50);
	 }

	//HAL_Delay(50);

  }
  /* USER CODE END start_rf */
}

/* USER CODE BEGIN Header_start_can */
/**
* @brief Function implementing the can thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_can */
void start_can(void *argument)
{
  /* USER CODE BEGIN start_can */
	int flag_can=0;
  /* Infinite loop */
  for(;;)
  {
	  if(!(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &pRxHeader, RxData))){
	  switch (pRxHeader.StdId) {
		case 37:
			ecar.battery_voltage[0]=RxData[0]*100+RxData[1];
			ecar.battery_voltage[1]=RxData[2]*100+RxData[3];
			ecar.battery_voltage[2]=RxData[4]*100+RxData[5];
			ecar.battery_voltage[3]=RxData[6]*100+RxData[7];
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			break;
		case 38:
			ecar.battery_voltage[4]=RxData[0]*100+RxData[1];
			ecar.battery_voltage[5]=RxData[2]*100+RxData[3];
			ecar.battery_voltage[6]=RxData[4]*100+RxData[5];
			ecar.battery_voltage[7]=RxData[6]*100+RxData[7];
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			break;
		case 39:
			ecar.battery_voltage[8]=RxData[0]*100+RxData[1];
			ecar.battery_voltage[9]=RxData[2]*100+RxData[3];
			ecar.battery_voltage[10]=RxData[4]*100+RxData[5];
			ecar.battery_voltage[11]=RxData[6]*100+RxData[7];
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			break;
		case 40:
			ecar.battery_voltage[12]=RxData[0]*100+RxData[1];
			ecar.battery_voltage[13]=RxData[2]*100+RxData[3];
			ecar.battery_voltage[14]=RxData[4]*100+RxData[5];
			ecar.battery_voltage[15]=RxData[6]*100+RxData[7];
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			break;
		case 41:
			ecar.battery_voltage[16]=RxData[0]*100+RxData[1];
			ecar.battery_voltage[17]=RxData[2]*100+RxData[3];
			ecar.battery_voltage[18]=RxData[4]*100+RxData[5];
			ecar.battery_voltage[19]=RxData[6]*100+RxData[7];
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			break;
		case 42:
			ecar.battery_voltage[20]=RxData[0]*100+RxData[1];
			ecar.battery_voltage[21]=RxData[2]*100+RxData[3];
			ecar.battery_voltage[22]=RxData[4]*100+RxData[5];
			ecar.battery_voltage[23]=RxData[6]*100+RxData[7];
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			break;
	  }
	  uint8_t can_feedback[1];can_feedback[0]=pRxHeader.StdId;
	  if(!(HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, can_feedback, &pTxMailbox))){
		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	  }

	}

		/*if(data_merge()){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,1);
			if ((pRxHeader.StdId == 1))
			{
			   ecar.battery_voltage[0]=converted_float[0];
			   ecar.battery_voltage[1]=converted_float[1];
			   ecar.battery_voltage[2]=converted_float[2];
			   ecar.battery_voltage[3]=converted_float[3];
			}
			if ((pRxHeader.StdId == 2))
			{
				ecar.battery_voltage[4]=converted_float[0];
				ecar.battery_voltage[5]=converted_float[1];
				ecar.battery_voltage[6]=converted_float[2];
				ecar.battery_voltage[7]=converted_float[3];
			}
			if ((pRxHeader.StdId == 3))
		    {
				ecar.battery_voltage[8]=converted_float[0];
				ecar.battery_voltage[9]=converted_float[1];
				ecar.battery_voltage[10]=converted_float[2];
				ecar.battery_voltage[11]=converted_float[3];
			}
			if ((pRxHeader.StdId == 259))
			{
				ecar.battery_voltage[12]=converted_float[0];
				ecar.battery_voltage[13]=converted_float[1];
				ecar.battery_voltage[14]=converted_float[2];
				ecar.battery_voltage[15]=converted_float[3];
			}
			if ((pRxHeader.StdId == 260))
			{
				ecar.battery_voltage[16]=converted_float[0];
				ecar.battery_voltage[17]=converted_float[1];
				ecar.battery_voltage[18]=converted_float[2];
				ecar.battery_voltage[19]=converted_float[3];
			}
			if ((pRxHeader.StdId == 261))
			{
				 ecar.battery_voltage[20]=converted_float[0];
				 ecar.battery_voltage[21]=converted_float[1];
				 ecar.battery_voltage[22]=converted_float[2];
				 ecar.battery_voltage[23]=converted_float[3];
			}
			HAL_Delay(20);
		}

		else{
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,0);
		}*/

   //flag_can++;
  }
  /* USER CODE END start_can */
}

/* USER CODE BEGIN Header_right_signal_start */
/**
* @brief Function implementing the right_signal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_right_signal_start */
void right_signal_start(void *argument)
{
  /* USER CODE BEGIN right_signal_start */
  /* Infinite loop */
  for(;;)
  {
	 while(ecar.RIGHT_SIGNAL_FLAG){
	    ecar.RIGHT_SIGNAL_FLAG=0;
		right_signal();
		osDelay(100);
	 }
  }
  /* USER CODE END right_signal_start */
}

/* USER CODE BEGIN Header_left_signal_signal */
/**
* @brief Function implementing the left_signal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_left_signal_signal */
void left_signal_signal(void *argument)
{
  /* USER CODE BEGIN left_signal_signal */
  /* Infinite loop */
  for(;;)
  {
	  while(ecar.LEFT_SIGNAL_FLAG){
	  	 ecar.LEFT_SIGNAL_FLAG=0;
	  	 left_signal();
	  	 osDelay(100);
	  }
  }
  /* USER CODE END left_signal_signal */
}

/* USER CODE BEGIN Header_stop_signal_start */
/**
* @brief Function implementing the stop_signal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_stop_signal_start */
void stop_signal_start(void *argument)
{
  /* USER CODE BEGIN stop_signal_start */
  /* Infinite loop */
  for(;;)
  {
	  while(ecar.STOP_SIGNAL_FLAG){
	  	 ecar.STOP_SIGNAL_FLAG=0;
	  	 stop_signal();
	  	 osDelay(100);
	  }
  }
  /* USER CODE END stop_signal_start */
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	datasentflag=1;
}

void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}



void Set_Brightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS

	if (brightness > 45) brightness = 45;
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j=1; j<4; j++)
		{
			float angle = 90-brightness;  // in degrees
			angle = angle*PI / 180;  // in rad
			LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
		}
	}

#endif

}

uint16_t pwmData[(24*MAX_LED)+50];

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
#if USE_BRIGHTNESS
		color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
#else
		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
#endif

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwmData[indx] = 70;  // 2/3 of 105
			}

			else pwmData[indx] = 35;  // 1/3 of 105

			indx++;
		}

	}

	for (int i=0; i<50; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
	while (!datasentflag){};
	datasentflag = 0;
}

void Reset_LED (void)
{
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Data[i][0] = i;
		LED_Data[i][1] = 0;
		LED_Data[i][2] = 0;
		LED_Data[i][3] = 0;
	}
}

void right_signal(){
	for(int i=RIGHT_SIGNAL;i<MAX_LED;i++){
			  Set_LED(((i-1)<RIGHT_SIGNAL?RIGHT_SIGNAL:MAX_LED-1), 255, 0, 0);
			  Set_LED(i, 255, 0, 0);
			  for(int j=RIGHT_SIGNAL;j<MAX_LED;j++){
				  if(j==i||j==i-1){
				  }else{
					  Set_LED(j, 0, 0, 0);
				  }
			  }
			  WS2812_Send();
			  HAL_Delay(45);
		  }
}

void stop_signal(){
	const int zer0=0;
	for(int i=STOP_SIGNAL;i<MAX_LED-l;i++){
		Set_LED(i, 0, 0, 255);
	}
	WS2812_Send();
	HAL_Delay(300);
	for(int i=STOP_SIGNAL;i<MAX_LED-l;i++){
			Set_LED(i, 0, 0, 0);
	}

}

void left_signal(){
	for(int i=MAX_LED-l-l;i>LEFT_SIGNAL;i--){
		Set_LED(((i-1)>LEFT_SIGNAL?LEFT_SIGNAL:i-1), 0, 0, 255);
		Set_LED(i, 255, 0, 0);
		for(int j=MAX_LED-l-l;j>LEFT_SIGNAL;j--){
		if(j==i){
		}else{
		    Set_LED(j, 0, 0, 0);
		}
		}
		WS2812_Send();
		HAL_Delay(100);
	}
}


float convert_float (int data_1, int data_2){
    return (float)(data_1 * 100  + data_2)/1000;
}

int data_merge(){

	if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &pRxHeader, RxData)){
		return 0;
	}
	converted_float[0] = convert_float(RxData[0], RxData[1]);
	converted_float[1] = convert_float(RxData[2], RxData[3]);
	converted_float[2] = convert_float(RxData[4], RxData[5]);
	converted_float[3] = convert_float(RxData[6], RxData[7]);
	return 1;
}

void toggle_func(GPIO_TypeDef * port,uint16_t pin,int togg_flag,const int const_val){
	if(togg_flag==const_val){
		port->ODR |= 1<<pin;
		togg_flag=0;
	}
	port->ODR &= ~(1<<pin);
}

int prepare_data(){
	sprintf(arr,"%c%c",ecar.direction,ci2a(ecar.speed_for_rf));
	for(int j=0;j<24;j++){
		strcat(arr,cf2a(ecar.battery_voltage[j],tempp[j]));
	}
	strcat(arr,ci2a(ecar.battery_temparature));
	strcat(arr,"1\n");
	return 1;
}
int k=32;
void deneme(){

	ecar.direction = '-';
	ecar.speed_for_rf = 25;
	ecar.battery_voltage[0] = 32.33;
	ecar.battery_voltage[1] = 34.35;
	ecar.battery_voltage[2] = 36.37;
	ecar.battery_voltage[3] = 38.39;
	ecar.battery_voltage[4] = 40.41;
		ecar.battery_voltage[5] = 42.43;
		ecar.battery_voltage[6] = 44.45;
		ecar.battery_voltage[7] = 46.47;
		ecar.battery_voltage[8] = 48.49;
			ecar.battery_voltage[9] = 50.51;
			ecar.battery_voltage[10] = 52.53;
			ecar.battery_voltage[11] = 54.55;
			ecar.battery_voltage[12] = 56.57;
				ecar.battery_voltage[13] = 58.59;
				ecar.battery_voltage[14] = 60.61;
				ecar.battery_voltage[15] = 62.63;
				ecar.battery_voltage[16] = 64.65;
							ecar.battery_voltage[17] = 66.67;
							ecar.battery_voltage[18] = 68.69;
							ecar.battery_voltage[19] = 70.71;
							ecar.battery_voltage[20] = 72.73;
								ecar.battery_voltage[21] = 74.75;
								ecar.battery_voltage[22] = 76.77;
								ecar.battery_voltage[23] = 78.79;
								ecar.battery_temparature=80;
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /*HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, m[cnt]);
   cnt++;
   if(cnt==(sizeof m / sizeof *m)-5){
 	  cnt=0;
   }*/
  /* USER CODE BEGIN Callback 1 */
 /* HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
  HAL_Delay(200);*/
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
