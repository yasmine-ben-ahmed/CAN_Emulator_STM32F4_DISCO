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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];

uint8_t canIdBuffer[4];
uint32_t receivedCanId;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void sendTestCANFrames(void);
void sendTestUartCANFrames(void);
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

  char msg[] = "Hello from STM32F407!\r\n";

  // Transmit message
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

  HAL_UART_Receive_IT(&huart2, canIdBuffer, 4);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
      // Check if User Button is pressed (active HIGH)
   /*   if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
      {
          // Simple debounce delay
          HAL_Delay(50);

          // Check again to confirm button press
          if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
          {
              //sendTestCANFrames(); // Call your function
              sendTestUartCANFrames();   // 🔥 Send the 3 test messages
              // Wait until button is released to avoid multiple triggers
              while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET);
          }
      }*/

  }
  /* USER CODE END 3 */
  }


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

  // Filter: accept everything
 /* CAN_FilterTypeDef filter;

  filter.FilterActivation = ENABLE;
  filter.FilterBank = 0;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterIdHigh = 0;
  filter.FilterIdLow = 0;
  filter.FilterMaskIdHigh = 0;
  filter.FilterMaskIdLow = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;

  HAL_CAN_ConfigFilter(&hcan1, &filter);
*/

  CAN_FilterTypeDef filter;
  // bank 0 -> ID1 & ID2
  uint32_t v1 = (0x1904001 << 3) | (1U<<2);
  uint32_t v2 = (0x1914001 << 3) | (1U<<2);

  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;

  filter.FilterIdHigh = (v1 >> 16) & 0xFFFF;
  filter.FilterIdLow  = v1 & 0xFFFF;
  filter.FilterMaskIdHigh = (v2 >> 16) & 0xFFFF; // second ID
  filter.FilterMaskIdLow  = v2 & 0xFFFF;

  HAL_CAN_ConfigFilter(&hcan1, &filter);

  // bank 1 -> ID3 only (put ID3 + a dummy (e.g. 0xFFFFFFFF))
  uint32_t v3 = (0x1924001 << 3) | (1U<<2);
  uint32_t dummy = 0xFFFFFFFF;

  filter.FilterBank = 1;
  filter.FilterIdHigh = (v3 >> 16) & 0xFFFF;
  filter.FilterIdLow  = v3 & 0xFFFF;
  filter.FilterMaskIdHigh = (dummy >> 16) & 0xFFFF;
  filter.FilterMaskIdLow  = dummy & 0xFFFF;

  HAL_CAN_ConfigFilter(&hcan1, &filter);




  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/***************receive real id from UART & send to f7 via can **********************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // Reconstruct 28-bit CAN ID
        uint32_t receivedCanId = ((uint32_t)canIdBuffer[0] << 24) |
                                 ((uint32_t)canIdBuffer[1] << 16) |
                                 ((uint32_t)canIdBuffer[2] << 8)  |
                                 ((uint32_t)canIdBuffer[3]);
        receivedCanId &= 0x0FFFFFFF;  // 28-bit mask

        // Blink LED: ON -> short delay -> OFF
        switch (receivedCanId)
        {
            case 0x1900140: // SOC, total voltage, current
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
                HAL_Delay(200);
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);


                break;

            case 0x1910140: // Max/Min cell voltages
               HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
               HAL_Delay(200);
               HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

                break;

            case 0x1920140: // Status charge/discharge
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
                HAL_Delay(200);
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);


                break;

            default:
                break;
        }

        // ----------------- CAN SEND -----------------
        CAN_TxHeaderTypeDef txHeader;
        uint8_t txData[8] = {0}; // your payload, e.g., zeros
        uint32_t txMailbox;

        txHeader.StdId = 0;           // not used for extended
        txHeader.ExtId = receivedCanId;
        txHeader.IDE = CAN_ID_EXT;    // extended frame
        txHeader.RTR = CAN_RTR_DATA;
        txHeader.DLC = 8;
        txHeader.TransmitGlobalTime = DISABLE;

        if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox) != HAL_OK)
        {
            // Error handling led red
        	 HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
        	  HAL_Delay(500);


        }

        // Restart UART interrupt for next ID
        HAL_UART_Receive_IT(&huart2, canIdBuffer, 4);
    }
}

/**********************receive real data from f7 via can & send to uart  ****************************/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);

    char uartMsg[150];


    if (rxHeader.IDE == CAN_ID_EXT)
    {
        switch (rxHeader.ExtId)
        {
            // -------------------------------------------------------
            // 1️⃣ Answer to ID 0x1900140 → F7 sends SOC, voltages, current
            // -------------------------------------------------------
            case 0x1904001:
            {
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
                HAL_Delay(100);
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);


                uint16_t cumVolt   = (rxData[0] << 8) | rxData[1];    // 0.1V
                uint16_t packVolt  = (rxData[2] << 8) | rxData[3];    // 0.1V
                int16_t  current   = (rxData[4] << 8) | rxData[5];    // Offset 30000 → 0.1A
                uint16_t SOC       = (rxData[6] << 8) | rxData[7];    // 0.1%

                current -= 30000; // remove offset

                snprintf(uartMsg, sizeof(uartMsg),
                    "\r\n[ID 0x1904001] SOC=%.1f%%  CumVolt=%.1fV  PackVolt=%.1fV  Current=%.1fA\r\n",
                    SOC/10.0, cumVolt/10.0, packVolt/10.0, current/10.0);



                HAL_UART_Transmit(&huart2, (uint8_t*)uartMsg, strlen(uartMsg), 500);
                break;
            }

            // -------------------------------------------------------
            // 2️⃣ Answer to ID 0x1910140 → Max/Min cell voltages
            // -------------------------------------------------------
            case 0x1914001:
            {
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
                HAL_Delay(100);
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);



                uint16_t maxV = (rxData[0] << 8) | rxData[1];
                uint8_t  maxCell = rxData[2];
                uint16_t minV = (rxData[3] << 8) | rxData[4];
                uint8_t  minCell = rxData[5];

                snprintf(uartMsg, sizeof(uartMsg),
                    "\r\n[ID 0x1914001] Max=%.3fV(Cell %d)  Min=%.3fV(Cell %d)\r\n",
                    maxV/1000.0, maxCell, minV/1000.0, minCell);

                HAL_UART_Transmit(&huart2, (uint8_t*)uartMsg, strlen(uartMsg), 500);
                break;
            }

            // -------------------------------------------------------
            // 3️⃣ Answer to ID 0x1920140 → temperatures
            // -------------------------------------------------------
            case 0x1924001:
            {
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
                HAL_Delay(100);
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

                uint8_t maxTemp = rxData[0] - 40;
                uint8_t maxTempCell = rxData[1];

                uint8_t minTemp = rxData[2] - 40;
                uint8_t minTempCell = rxData[3];

                snprintf(uartMsg, sizeof(uartMsg),
                    "\r\n[ID 0x1924001] MaxTemp=%dC(Cell %d)  MinTemp=%dC(Cell %d)\r\n",
                    maxTemp, maxTempCell, minTemp, minTempCell);

                HAL_UART_Transmit(&huart2, (uint8_t*)uartMsg, strlen(uartMsg), 500);
                break;
            }

            default:
                break;
        }
    }
}



/**********************TEST F4 TO F7 WITHOUT UART*********************/

void sendTestCANFrames(void)
{
    uint32_t testIds[3] = {0x1900140, 0x1910140, 0x1920140};
    uint8_t txData[8] = {0}; // payload zeros
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;

    for(int i = 0; i < 3; i++)
    {
        txHeader.StdId = 0;           // Not used for extended
        txHeader.ExtId = testIds[i];  // 28-bit extended ID
        txHeader.IDE = CAN_ID_EXT;
        txHeader.RTR = CAN_RTR_DATA;
        txHeader.DLC = 8;
        txHeader.TransmitGlobalTime = DISABLE;

        if(HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox) == HAL_OK)
        {
            // LED blink to indicate frame sent
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

        } else {
            // Error handling led red
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
            HAL_Delay(500);
        }

        HAL_Delay(500); // wait 0.5s between frames
    }
}

/*******************TEST F4 DATA FROMF7 TO UART********************************/
void sendTestUartCANFrames(void)
{
    char uartMsg[120];

    // 1️⃣ ID 0x1904001
    snprintf(uartMsg, sizeof(uartMsg),
        "\r\n[ID 0x1904001] SOC=%.1f%%  CumVolt=%.1fV  PackVolt=%.1fV  Current=%.1fA\r\n",
        75.3f, 387.0f, 385.5f, -12.4f);
    HAL_UART_Transmit(&huart2, (uint8_t*)uartMsg, strlen(uartMsg), 1000); // big timeout
    HAL_Delay(50);

    // 2️⃣ ID 0x1914001
    snprintf(uartMsg, sizeof(uartMsg),
        "\r\n[ID 0x1914001] Max=%.3fV(Cell %d)  Min=%.3fV(Cell %d)\r\n",
        4.123f, 7, 3.987f, 12);
    HAL_UART_Transmit(&huart2, (uint8_t*)uartMsg, strlen(uartMsg), 1000);
    HAL_Delay(50);

    // 3️⃣ ID 0x1924001
    snprintf(uartMsg, sizeof(uartMsg),
        "\r\n[ID 0x1924001] MaxTemp=%dC(Cell %d)  MinTemp=%dC(Cell %d)\r\n",
        45, 3, 21, 10);
    HAL_UART_Transmit(&huart2, (uint8_t*)uartMsg, strlen(uartMsg), 1000);
    HAL_Delay(50);
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
