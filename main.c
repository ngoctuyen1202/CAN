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
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t uart3_receive;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_TxHeaderTypeDef CAN1_pHeader;
CAN_RxHeaderTypeDef CAN1_pHeaderRx;
CAN_FilterTypeDef CAN1_sFilterConfig;
CAN_TxHeaderTypeDef CAN2_pHeader;
CAN_RxHeaderTypeDef CAN2_pHeaderRx;
CAN_FilterTypeDef CAN2_sFilterConfig;
uint32_t CAN1_pTxMailbox;
uint32_t CAN2_pTxMailbox;

uint16_t NumBytesReq = 0;
uint8_t  REQ_BUFFER  [4096];
uint8_t  REQ_1BYTE_DATA;

uint8_t CAN1_DATA_TX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t CAN1_DATA_RX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t CAN2_DATA_TX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t CAN2_DATA_RX[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t test[7] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t mes_first[3] = {0x00,0xA2,0x00};
char tb1[] = "CAN 1 TX\n";
char tb2[] = "CAN 2 TX\n";
char tb3[] = "CAN 1 RX\n";
char tb4[] = "CAN 2 RX\n";
uint16_t Num_Consecutive_Tester;
uint8_t  Flg_Consecutive = 0;

unsigned int TimeStamp;
// maximum characters send out via UART is 30
char bufsend[30]="XXX: D1 D2 D3 D4 D5 D6 D7 D8  ";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void MX_CAN1_Setup();
void MX_CAN2_Setup();
void USART3_SendString(uint8_t *ch);
void PrintCANLog(uint16_t CANID, uint8_t * CAN_Frame);
void SID_22_Practice();
void SID_2E_Practice();
void SID_27_Practice();
void delay(uint16_t delay);
// bo sung
// NGAT NHAN DU LIEU
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  if (hcan->Instance == CAN2)
  {
	  // CAN 2 RX
  USART3_SendString((uint8_t*)tb4);
  HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &CAN2_pHeaderRx, CAN2_DATA_RX);
  PrintCANLog(CAN2_pHeaderRx.StdId, &CAN2_DATA_RX[0]);
     // Lấy các giá trị nhận được và chuyển vào CAN2 TX
  memcpy(CAN2_DATA_TX, CAN2_DATA_RX, 6);
   // byte thứ 2 sẽ bằng byte thứ 0 + byte thứ 1
  CAN2_DATA_TX[2] = CAN2_DATA_TX[0] + CAN2_DATA_TX[1];
 }
   if (hcan->Instance == CAN1)
  {
	   // CAN 1 RX
	  USART3_SendString((uint8_t*)tb3);
	  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_pHeaderRx, CAN1_DATA_RX);
	  PrintCANLog(CAN1_pHeaderRx.StdId, &CAN1_DATA_RX[0]);
	  //memcpy(CAN1_DATA_TX, CAN1_DATA_RX, 6);
	 //CAN1_DATA_TX[2] = CAN1_DATA_TX[0] + CAN1_DATA_TX[1];
  }
 }

// TINH TOAN CRC
int calc_SAE_J1850(int data[], int Crc_len)
{
	int idx, crc, temp, temp1, temp2, idy;
	crc = 0;
	idx = 0;
	idy = 0;
	temp = 0;
	temp1 = 0;
	temp2 = 0;
	for (idx=0; idx < Crc_len+1;idx++){
		if (idx == 0)
		{
			temp1 = 0;
		}
		else
		{
			temp1 = data[Crc_len -idx];
		}
		crc = (crc ^ temp1);
		for (idy=8; idy>0; idy--){
			// Save the value before the top bit is shifted out
			temp2 = crc;
			crc<<=1;
			if (0 != (temp2 & 128))
			{
				crc ^= 0X1D;
			}
		}
	}
	return crc;
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
	uint16_t i,j = 0;
	uint16_t Consecutive_Cntr = 0;
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
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // Ban tin truyen 1
  CAN1_pHeader.DLC = 8; // data length
  CAN1_pHeader.IDE = CAN_ID_STD;
  CAN1_pHeader.StdId= 0x012;
  CAN1_pHeader.RTR = CAN_RTR_DATA;
  CAN1_pHeader.TransmitGlobalTime = DISABLE;
  CAN1_pHeader.ExtId = 0;
  CAN1_DATA_TX[1] = 0xA2;
  CAN1_DATA_TX[6] = 0x00;

  // Ban tin truyen 2
  CAN2_pHeader.DLC = 8; // data length
  CAN2_pHeader.IDE = CAN_ID_STD;
  CAN2_pHeader.StdId= 0x0A2;
  CAN2_pHeader.RTR = CAN_RTR_DATA;
  CAN2_pHeader.TransmitGlobalTime = DISABLE;
  CAN2_pHeader.ExtId = 0;
  CAN2_DATA_TX[6] = 0x00;

  //  Applying configurations and Start CAN
  MX_CAN1_Setup();
  MX_CAN2_Setup();
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Example Function to print can message via uart
  //PrintCANLog(CAN1_pHeader.StdId, &CAN1_DATA_TX[0]);
// Việc truyền liên tục
   while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	   // byte 6 lưu stt gửi đi khi đến giá trị 15 thì reset lại bằng 0
  	  if (CAN1_DATA_TX[6] == 0x10)
	  {
		  CAN1_DATA_TX[6] = 0;
	  }

	  else{
	memcpy(test, CAN1_DATA_TX, 7);
	// tính toán CRC
	CAN1_DATA_TX[7]= (uint8_t)calc_SAE_J1850(test,7);
	// CAN1 TX
	USART3_SendString((uint8_t*)tb1);
	PrintCANLog(CAN1_pHeader.StdId, &CAN1_DATA_TX[0]);

	HAL_CAN_AddTxMessage(&hcan1, &CAN1_pHeader, CAN1_DATA_TX, CAN1_pTxMailbox);
	HAL_Delay(2000);
	  }
  	// byte 6 lưu stt gửi đi khi đến giá trị 15 thì reset lại bằng 0
	 if (CAN2_DATA_TX[6] == 0x10)
		  {
			  CAN2_DATA_TX[6] = 0;
		  }

	 else{
		 // tính toán CRC
	memcpy(test, CAN1_DATA_TX, 7);
    CAN2_DATA_TX[7]= (uint8_t)calc_SAE_J1850(test,7);
    // CAN2 TX
	USART3_SendString((uint8_t*)tb2);
	PrintCANLog(CAN2_pHeader.StdId, &CAN2_DATA_TX[0]);
	HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, CAN2_pTxMailbox);
	HAL_Delay(2000);
	  }
	 // Byte thứ 6 lưu giá trị stt gửi sau mỗi lần gửi tăng lên 1
	CAN1_DATA_TX[6] = CAN1_DATA_TX[6]+1;
	CAN2_DATA_TX[6] = CAN2_DATA_TX[6]+1;
	memcpy(CAN1_DATA_TX, mes_first,3);
    if(!BtnU) /*IG OFF->ON stimulation*/
    {
      delay(20);
      USART3_SendString((uint8_t *)"IG OFF ");
      while(!BtnU);
      MX_CAN1_Setup();
      MX_CAN2_Setup();
      USART3_SendString((uint8_t *)"-> IG ON \n");
      delay(20);
    }
  }

  memset(&REQ_BUFFER,0x00,4096);
  NumBytesReq = 0;

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
  RCC_OscInitStruct.PLL.PLLN = 80;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
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
//  CAN1_sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
//  CAN1_sFilterConfig.FilterBank = 0;
//  CAN1_sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//  CAN1_sFilterConfig.FilterIdHigh = (0x0A2) << 5;
//  CAN1_sFilterConfig.FilterIdLow = 0;
//  CAN1_sFilterConfig.FilterMaskIdHigh = 0x00;
//  CAN1_sFilterConfig.FilterMaskIdLow = 0;
//  CAN1_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
//  CAN1_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//  CAN1_sFilterConfig.SlaveStartFilterBank = 14;
    CAN1_sFilterConfig.FilterBank = 0;
    CAN1_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN1_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN1_sFilterConfig.FilterIdHigh = (0x0A2) << 5;
    CAN1_sFilterConfig.FilterIdLow = 0x0000;
    CAN1_sFilterConfig.FilterMaskIdHigh = (0x0A2) << 5;
    CAN1_sFilterConfig.FilterMaskIdLow = 0x0000;
    CAN1_sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    CAN1_sFilterConfig.FilterActivation = ENABLE;
    CAN1_sFilterConfig.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &CAN1_sFilterConfig) != HAL_OK)
      {
        /* Filter configuration Error */
        Error_Handler();
      }
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 5;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
//    CAN2_sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
//    CAN2_sFilterConfig.FilterBank = 14;
//    CAN2_sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//    CAN2_sFilterConfig.FilterIdHigh = (0x012) << 5;
//    CAN2_sFilterConfig.FilterIdLow = 0;
//    CAN2_sFilterConfig.FilterMaskIdHigh = 0x00;
//    CAN2_sFilterConfig.FilterMaskIdLow = 0;
//    CAN2_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
//    CAN2_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//    CAN2_sFilterConfig.SlaveStartFilterBank = 14;
      CAN2_sFilterConfig.FilterBank = 14;
      CAN2_sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
      CAN2_sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
      CAN2_sFilterConfig.FilterIdHigh = (0x012) << 5;
      CAN2_sFilterConfig.FilterIdLow = 0x0000;
      CAN2_sFilterConfig.FilterMaskIdHigh = (0x012) << 5;
      CAN2_sFilterConfig.FilterMaskIdLow = 0x0000;
      CAN2_sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
      CAN2_sFilterConfig.FilterActivation = ENABLE;
      CAN2_sFilterConfig.SlaveStartFilterBank = 14;
      if (HAL_CAN_ConfigFilter(&hcan2, &CAN2_sFilterConfig) != HAL_OK)
            {
              /* Filter configuration Error */
              Error_Handler();
            }
  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC4 PC5 PC6
                           PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void MX_CAN1_Setup()
{
	HAL_CAN_ConfigFilter(&hcan1, &CAN1_sFilterConfig);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}
void MX_CAN2_Setup()
{
	HAL_CAN_ConfigFilter(&hcan2, &CAN2_sFilterConfig);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void USART3_SendString(uint8_t *ch)
{
   while(*ch!=0)
   {
      HAL_UART_Transmit(&huart3, ch, 1,HAL_MAX_DELAY);
      ch++;
   }
}
void PrintCANLog(uint16_t CANID, uint8_t * CAN_Frame)
{
	uint16_t loopIndx = 0;
	char bufID[3] = "   ";
	char bufDat[2] = "  ";
	char bufTime [8]="        ";

	sprintf(bufTime,"%d",TimeStamp);
	USART3_SendString((uint8_t*)bufTime);
	USART3_SendString((uint8_t*)" ");

	sprintf(bufID,"%3X",CANID);
	for(loopIndx = 0; loopIndx < 3; loopIndx ++)
	{
		bufsend[loopIndx]  = bufID[loopIndx];
	}
	bufsend[3] = ':';
	bufsend[4] = ' ';


	for(loopIndx = 0; loopIndx < 8; loopIndx ++ )
	{
		sprintf(bufDat,"%02X",CAN_Frame[loopIndx]);
		bufsend[loopIndx*3 + 5] = bufDat[0];
		bufsend[loopIndx*3 + 6] = bufDat[1];
		bufsend[loopIndx*3 + 7] = ' ';
	}
	bufsend[29] = '\n';
	USART3_SendString((unsigned char*)bufsend);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	REQ_BUFFER[NumBytesReq] = REQ_1BYTE_DATA;
	NumBytesReq++;
	//REQ_BUFFER[7] = NumBytesReq;
}
void delay(uint16_t delay)
{
	HAL_Delay(delay);
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
