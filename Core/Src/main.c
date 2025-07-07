/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cli.h"
#include "vesc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define R 340

#define HeadLight 0
#define Stop 1
#define Reverse 2
#define Buzzer 3
#define Mayak 4

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/*---------------------- Init CAN-----------------------*/

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxBufCan[8];
uint8_t RxBufCan[16];
uint32_t TxMailbox = 0;

CAN_FilterTypeDef  sFilterConfig;

/*-----------------------------*/
CliType cli;
QueryType query;
uint8_t DataInput[256];
uint8_t DataOutput[256];
uint8_t DataInUart;//переменная для приема байта по UART

uint8_t Transmit(uint8_t* Data, uint8_t len) {
  HAL_UART_Transmit(&huart2, Data, len, len*5);
}

CmdType cmd[] = {{"hello", HelloWorld}
                ,{"set_vesc_id", SetVescID}
                ,{"set_vesc_erpm", SetVescERPM}
                ,{"set_motor_speed", SetSpeedWheel}
                ,{"vel", SetBodyVel}
                ,{"alarm", SetLedsAndAlarm}
};

uint8_t HelloWorld(int32_t* Pars) {
  HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
  print(&cli, "Hello World!!! My params is %d", *Pars);
  }

  uint8_t VescID[6];
  int16_t VescERPM[6];
  int8_t VescDir[6];

uint8_t SetVescID(int32_t* Pars) {
  VescID[Pars[0]] = (uint8_t)Pars[1];
  print(&cli, "Setted VESC ID %d", VescID);
}

uint8_t SetVescERPM(int32_t* Pars) {
  VescERPM[Pars[0]] = (int16_t)Pars[1];
  print(&cli, "Setted VESC ERPM %d", VescERPM);
}

uint8_t SetBodyVel(int32_t* Pars) {
  int32_t Data[4];
  for(uint8_t i = 0; i < 3; i++) {
    Data[0] = i;
    Data[1] = Pars[0] + Pars[1];
    SetSpeedWheel(Data);
  }
  for(uint8_t i = 3; i < 6; i++) {
    Data[0] = i;
    Data[1] = Pars[0] - Pars[1];
    SetSpeedWheel(Data);
  }
}

uint8_t SetSpeedWheel(int32_t* Pars) {
  VescERPM[Pars[0]] = (int16_t)(Pars[1] * 1.67467);
  print(&cli, "Setted speed for motor %d of %d", Pars[0], VescERPM[Pars[0]]);
}
uint8_t SetLedsAndAlarm(int32_t* Pars) {
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, (Pars[0] & (1 << Mayak)) >> Mayak);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, (Pars[0] & (1 << Buzzer)) >> Buzzer);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, (Pars[0] & (1 << Stop)) >> Stop);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, (Pars[0] & (1 << HeadLight)) >> HeadLight);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (Pars[0] & (1 << Reverse)) >> Reverse);

  print(&cli, "Leds setted: %d %d %d %d %d", (Pars[0] & (1 << HeadLight)) >> HeadLight, (Pars[0] & (1 << Stop)) >> Stop, (Pars[0] & (1 << Reverse)) >> Reverse, (Pars[0] & (1 << Buzzer)) >> Buzzer, (Pars[0] & (1 << Mayak)) >> Mayak);
}

can_vesc_t can_vesc_bus = {CanVescTransmit};

void CanVescTransmit(uint32_t id, const uint8_t *data, uint8_t len) {
  TxHeader.ExtId = id;
  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox);
}
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  cli.transmit = Transmit;
cli.InputCnt = DataInput;
cli.OutputCnt = DataOutput;
cli.Cmds = cmd;
cli.flag |= echo;
cli.query = &query;
cli.LengthQuery = 256;
cli.Ncmd = sizeof(cmd) / sizeof(cmd[0]);

HAL_UART_Receive_IT(&huart2, &DataInUart, 1);

  /*----------------- Setting CAN -----------------*/
 //can_transmit_eid = CanVescTransmit;
 TxHeader.StdId = 0x117;
 TxHeader.ExtId = 0x117;
 TxHeader.IDE = CAN_ID_EXT;
 TxHeader.RTR = CAN_RTR_DATA;
 TxHeader.DLC = 4;
 TxHeader.TransmitGlobalTime = DISABLE;

 sFilterConfig.FilterBank = 0;
 sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
 sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; 
 sFilterConfig.FilterIdHigh = (uint16_t)(0x00000917 >> 13);
 sFilterConfig.FilterIdLow = (uint16_t)(0x00000917 << 3) | 0x04;
 sFilterConfig.FilterMaskIdHigh = 0x0000;
 sFilterConfig.FilterMaskIdLow = 0x0000;
 sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
 sFilterConfig.FilterActivation = ENABLE;
 //sFilterConfig.SlaveStartFilterBank = 14;

 TxBufCan[0] = 1;
 TxBufCan[1] = 2;
 TxBufCan[2] = 3;
 TxBufCan[3] = 4;

 HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

 HAL_CAN_Start(&hcan1);
 //HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
 VescID[0] = 11;
 VescID[1] = 10;
 VescID[2] = 12;
 VescID[3] = 14;
 VescID[4] = 15;
 VescID[5] = 16;

 VescERPM[0] = 0;
 VescERPM[1] = 0;
 VescERPM[2] = 0;
 VescERPM[3] = 0;
 VescERPM[4] = 0;
 VescERPM[5] = 0;

 VescDir[0] = 1;
 VescDir[1] = -1;
 VescDir[2] = -1;
 VescDir[3] = 1;
 VescDir[4] = -1;
 VescDir[5] = 1;

 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);

 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
 HAL_Delay(150);
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
 HAL_Delay(200);
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
 HAL_Delay(150);
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(20);

    // while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
    // HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxBufCan, &TxMailbox);
    for(uint8_t i = 0; i < 6; i++) {
      comm_can_set_rpm(VescID[i], VescERPM[i] * VescDir[i], &can_vesc_bus);
    }
    comm_can_set_rpm(VescID[0], VescERPM[0], &can_vesc_bus);
    if((cli.flag & busy) != 0) {
      CMDProcessing(&cli);
      cli.flag &= ~busy;
    }



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
  RCC_OscInitStruct.PLL.PLLN = 160;
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
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart) {
  //if(huart->Instance == USART1) {
    ProcessingInputData(&cli, DataInUart);
    HAL_UART_Receive_IT(&huart2, &DataInUart, 1);
  //}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxBufCan) == HAL_OK)
    // {
    //   print(&cli, "ERPM %d  Current %d Duty %d", (int32_t)(RxBufCan[0]<<32+RxBufCan[1]<<16+RxBufCan[2]<<8+RxBufCan[3]), (int16_t)(RxBufCan[4]<<8+RxBufCan[5]), (int16_t)(RxBufCan[6]<<8+RxBufCan[7]));
    //    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  
    // }
}
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
