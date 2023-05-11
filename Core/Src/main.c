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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "can-ids/CAN.h"
#include "can_serializer.h"
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
CAN_HandleTypeDef hcan;

CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
struct XBEE_TxBuffer {
  CANFrame frames[MAX_UNIQUE_IDS];
  uint32_t num_frames;
};
struct XBEE_TxBuffer xbee_tx_buffers[2];
struct XBEE_TxBuffer *xbee_active_buffer;
uint8_t active_buffer_idx = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// build and send a CAN frame from a received buffer
CAN_TxHeaderTypeDef outmsg;
void buildFrame(CANFrame *tempFrame)
{
  if (tempFrame->syncWord == 0xA55A && validateChecksum(tempFrame)) // got a valid frame
  {
    // build an Mbed CAN frame and send it
    outmsg.DLC = tempFrame->dlc;
    outmsg.RTR = CAN_RTR_DATA;
    
    if (tempFrame->extended)
    {
      outmsg.IDE = CAN_ID_EXT;
      outmsg.ExtId = tempFrame->id;
    }
    else
    {
      outmsg.IDE = CAN_ID_STD;
      outmsg.StdId = tempFrame->id;
    }

    uint32_t _mb;
    HAL_CAN_AddTxMessage(&hcan, &outmsg, tempFrame->data, &_mb);
  }
}


void updateCanMap(CAN_RxHeaderTypeDef *msg, uint8_t data[8]) {
  uint32_t id = msg->StdId; // TODO Support extended ids
  // check for ID in map
  for(uint32_t i = 0; i < xbee_active_buffer->num_frames; ++i){
    if(xbee_active_buffer->frames[i].id == id) {
      // update message
      xbee_active_buffer->frames[i].extended = 0;
      xbee_active_buffer->frames[i].dlc = msg->DLC;
      memcpy(xbee_active_buffer->frames[i].data, data, msg->DLC);

      fillChecksum(&xbee_active_buffer->frames[i]);
      return;
    }
  }

  // new message
  if(xbee_active_buffer->num_frames < MAX_UNIQUE_IDS) {
    xbee_active_buffer->frames[xbee_active_buffer->num_frames].id = msg->StdId;
    xbee_active_buffer->frames[xbee_active_buffer->num_frames].extended = 0;
    xbee_active_buffer->frames[xbee_active_buffer->num_frames].dlc = msg->DLC;
    memcpy(xbee_active_buffer->frames[xbee_active_buffer->num_frames].data, data, msg->DLC);

      fillChecksum(&xbee_active_buffer->frames[xbee_active_buffer->num_frames]);
    xbee_active_buffer->num_frames++;
  }
}


// CAN Interrupt handler. Called whenever a new CAN frame is received.
void can_irq(CAN_HandleTypeDef *pcan) {
  // Message header
  CAN_RxHeaderTypeDef msg;
  // Message data. Only filled with as much data as the message contained, the rest is garbage.
  uint8_t data[8];

  // Read the next CAN frame
  HAL_CAN_GetRxMessage(pcan, CAN_RX_FIFO0, &msg, data);

  // Only accept standard CAN IDs, we don't use extended IDs.
  if(msg.IDE == CAN_ID_STD) { 
    switch (msg.StdId)
      {
      // Add cases for each CAN ID you want to handle

      // ID 0xB0 is a bootloader command, so reset the mcu when received.
      case BOOTLOADER_ID:
        __NVIC_SystemReset(); // Reset to bootloader
        break;
      
      default:
        updateCanMap(&msg, data);
        break;
      }
  }
}



void xbee_tx(void) {
  struct XBEE_TxBuffer *send_buffer = xbee_active_buffer;
  
  __disable_irq();
  active_buffer_idx = (active_buffer_idx + 1) % 2;
  xbee_active_buffer = &xbee_tx_buffers[active_buffer_idx];
  xbee_active_buffer->num_frames = 0;
  __enable_irq();

  // check if there's anything to send
  if(send_buffer->num_frames == 0) return;

  // send frames 16 at a time
  for(uint32_t start_idx = 0; start_idx < send_buffer->num_frames; start_idx += 16) {
    // send up to 16 frames, but fewer if there's not enough
    uint32_t num_frames_to_send = 16;
    if(send_buffer->num_frames - start_idx < num_frames_to_send) num_frames_to_send = send_buffer->num_frames - start_idx;

    uint8_t *txdata = (uint8_t*)(void*) &send_buffer->frames[start_idx];
    HAL_UART_Transmit(&huart2, txdata, num_frames_to_send * sizeof(CANFrame), 100);
    HAL_Delay(INTERFRAME_PERIOD_MS);
  }
}
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
  MX_CAN_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  xbee_active_buffer = &xbee_tx_buffers[active_buffer_idx];

  // LED Indicator for startup
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);

  // wait for power to stabilize
  HAL_Delay(250);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(TRANSMIT_INTERVAL_MS);
    xbee_tx();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  // Set up filter to accept all CAN messages
  CAN_FilterTypeDef sf;
  sf.FilterMaskIdHigh = 0x0000;
  sf.FilterMaskIdLow = 0x0000;
  sf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sf.FilterBank = 0;
  sf.FilterMode = CAN_FILTERMODE_IDMASK;
  sf.FilterScale = CAN_FILTERSCALE_32BIT;
  sf.FilterActivation = CAN_FILTER_ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan, &sf) != HAL_OK) {
    Error_Handler();
  }
  
  // Sets up a callback to can_irq(...) which will be called each time a CAN message is received.
  if (HAL_CAN_RegisterCallback(&hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, can_irq)) {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    Error_Handler();
  }

  // Enables interrupts
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END CAN_Init 2 */
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */
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
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

#ifdef USE_FULL_ASSERT
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
