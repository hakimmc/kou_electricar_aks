/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "time.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
 ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId can_taskHandle;
osThreadId rf_taskHandle;
osThreadId gps_taskHandle;
osThreadId calculate_speedHandle;
/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef pTxHeader;
CAN_RxHeaderTypeDef pRxHeader;
CAN_FilterTypeDef sFilterConfig;
uint32_t pTxMailbox;
uint8_t can_array[8];
time_t start,stop;
uint8_t adc_value;
struct VARIABLES{
	char* char_speed;
	float speed;
	char* char_left_motor_voltage;
	float left_motor_voltage;
	char* char_left_motor_temparature;
	float left_motor_temparature;
	char* char_right_motor_voltage;
	float right_motor_voltage;
	char* char_right_motor_temparature;
	float right_motor_temparature;
	char* char_battery_voltage[4];
	float* battery_voltage[4];
	char* char_battery_temparature;
	float battery_temparature;
	int ADC_FLAG;
	int VAR_FLAG;
	float wheel_radius; // 0.5 meter
	int ADC_AVAILABLE;
	int ADC_CALCULATING;
	int CAN_FLAG;
}*ecar;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void can_start(void const * argument);
void rf_start(void const * argument);
void gps_start(void const * argument);
void speed_start(void const * argument);

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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  ecar->wheel_radius = 0.2932;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan1);

      HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  // enable interrupt

      pTxHeader.DLC = 1;
      pTxHeader.IDE = CAN_ID_STD;
      pTxHeader.RTR = CAN_RTR_DATA;
      pTxHeader.StdId = 0x012;

      //set filter parameters
      sFilterConfig.FilterActivation = ENABLE;
      sFilterConfig.FilterBank = 0;
      sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
      sFilterConfig.FilterIdHigh = 0x0000;
      sFilterConfig.FilterIdLow = 0x0000;
      sFilterConfig.FilterMaskIdHigh = 0x0000 << 5;
      sFilterConfig.FilterMaskIdLow = 0x0000 << 5;
      sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
      sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

      HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

  /* USER CODE END 2 */

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
  /* definition and creation of can_task */
  osThreadDef(can_task, can_start, osPriorityNormal, 0, 128);
  can_taskHandle = osThreadCreate(osThread(can_task), NULL);

  /* definition and creation of rf_task */
  osThreadDef(rf_task, rf_start, osPriorityIdle, 0, 128);
  rf_taskHandle = osThreadCreate(osThread(rf_task), NULL);

  /* definition and creation of gps_task */
  osThreadDef(gps_task, gps_start, osPriorityIdle, 0, 128);
  gps_taskHandle = osThreadCreate(osThread(gps_task), NULL);

  /* definition and creation of calculate_speed */
  osThreadDef(calculate_speed, speed_start, osPriorityIdle, 0, 128);
  calculate_speedHandle = osThreadCreate(osThread(calculate_speed), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_can_start */
/**
  * @brief  Function implementing the can_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_can_start */
void can_start(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &pRxHeader, can_array );

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_rf_start */
/**
* @brief Function implementing the rf_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rf_start */
void rf_start(void const * argument)
{
  /* USER CODE BEGIN rf_start */
  /* Infinite loop */
  for(;;)
  {
	if(ecar->CAN_FLAG && ecar->VAR_FLAG){
		char *dizi;
		dizi = (char*)malloc(256);
		sprintf(dizi,"%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f!%.2f",ecar->left_motor_voltage,ecar->left_motor_temparature,ecar->right_motor_voltage,ecar->left_motor_temparature,ecar->speed,ecar->battery_voltage[0],ecar->battery_voltage[1],ecar->battery_voltage[2],ecar->battery_voltage[3],ecar->battery_voltage[4],ecar->battery_voltage[5],ecar->battery_voltage[6],ecar->battery_voltage[7],ecar->battery_voltage[8],ecar->battery_voltage[9],ecar->battery_voltage[10],ecar->battery_voltage[11],ecar->battery_voltage[12],ecar->battery_voltage[13],ecar->battery_voltage[14],ecar->battery_voltage[16],ecar->battery_voltage[17],ecar->battery_voltage[18],ecar->battery_voltage[19],ecar->battery_voltage[20],ecar->battery_voltage[21],ecar->battery_voltage[22],ecar->battery_voltage[23],ecar->battery_temparature);
		HAL_UART_Transmit(&huart1, dizi, strlen(dizi), HAL_MAX_DELAY);
		free(dizi);
	}
    osDelay(1);
  }
  /* USER CODE END rf_start */
}

/* USER CODE BEGIN Header_gps_start */
/**
* @brief Function implementing the gps_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gps_start */
void gps_start(void const * argument)
{
  /* USER CODE BEGIN gps_start */
  /* Infinite loop */
  for(;;)
  {
	//just split for can variables
	//
  //V

	/*bms receiving*/
	  int truther = 1;
	switch(pRxHeader.StdId){
	case 0x101:
		ecar->battery_voltage[0] = &can_array;
		truther = truther<<1;
		break;
	case 0x102:
		ecar->battery_voltage[1] = &can_array;
		truther = truther<<1;
		break;
	case 0x103:
		ecar->battery_voltage[2] = &can_array;
		truther = truther<<1;
		break;
	case 0x104:
		ecar->battery_voltage[3] = &can_array;
		ecar->battery_temparature =
		truther = truther<<1;
		break;
	}
	if(truther==16) ecar->VAR_FLAG=1;
	else ecar->VAR_FLAG=0;
    osDelay(1);
  }
  /* USER CODE END gps_start */
}

/* USER CODE BEGIN Header_speed_start */
/**
* @brief Function implementing the calculate_speed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_speed_start */
void speed_start(void const * argument)
{
  /* USER CODE BEGIN speed_start */
  /* Infinite loop */
  for(;;)
  {
	  if(ecar->ADC_FLAG == ecar->ADC_AVAILABLE){
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, 1);
		  adc_value = HAL_ADC_GetValue(&hadc1);
		  if(adc_value>1024){
			  ecar->ADC_FLAG=2;
			  time(&start);
		  }
	  }
	  if(ecar->ADC_CALCULATING == ecar->ADC_FLAG){
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, 1);
		  int tempvalue = HAL_ADC_GetValue(&hadc1);
		  if(tempvalue==adc_value){
			  time(&stop);
			  ecar->speed = (2*3.14*ecar->wheel_radius)/difftime(stop, start);
			  ecar->ADC_FLAG=1;
		  }
		  continue;
	  }
	  HAL_ADC_Stop(&hadc1);
    osDelay(1);
  }
  /* USER CODE END speed_start */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
