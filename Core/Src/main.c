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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t rxBuffer[RX_BUFFER_SIZE]; // DMA buffer
uint8_t packetBuffer[RX_BUFFER_SIZE]; // Temporary buffer for packets
uint8_t packetIndex = 0; // Index for capturing packets
uint8_t expectedPacketLength = 0; // Expected full packet length

typedef struct {
	uint16_t values[16];
	uint32_t lastUpdateTime;
} RCChannels_t;

/* `rcChannels.values[<index>]` can be accessed to get the current
 * value of any channel at any given moment.
 * Example:
 * 		printf("%d\n", rcChannels.values[2]); // Print the value of 3rd channel
 */
static RCChannels_t rcChannels = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void parseCRSFPacket(uint8_t *packet, uint8_t length);
void parseRCChannels(uint8_t *payload);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		for (int i = 0; i < RX_BUFFER_SIZE; i++) {
			uint8_t byte = rxBuffer[i];

			if (packetIndex == 0) {
				if (byte == 0xC8) {  // Sync Byte
					packetBuffer[packetIndex++] = byte;
				}
			} else if (packetIndex == 1) {
				expectedPacketLength = byte + 2; // Total packet size
				packetBuffer[packetIndex++] = byte;

				if (expectedPacketLength > RX_BUFFER_SIZE) {
					packetIndex = 0;
				}
			} else {
				packetBuffer[packetIndex++] = byte;

				if (packetIndex == expectedPacketLength) {
					parseCRSFPacket(packetBuffer, packetIndex);
					packetIndex = 0;
				}
			}
		}

    HAL_UART_Receive_DMA(&huart1, rxBuffer, RX_BUFFER_SIZE);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		HAL_UART_Receive_DMA(&huart1, rxBuffer, RX_BUFFER_SIZE);
	}
}

void parseCRSFPacket(uint8_t *packet, uint8_t length) {
	// Invalid packet
	if (length < 3) return;

	uint8_t type = packet[2];  // Type index = 2
	uint8_t *payload = &packet[3];  // Payload index = 3

	switch (type) {
		case 0x16:  // RC Channels Data
			parseRCChannels(payload);
			break;
		default:
			break;
	}
}

void parseRCChannels(uint8_t *payload) {
	// Correct 11-bit extraction using bit shifts
	rcChannels.values[0]  = (payload[0]  | (payload[1]  << 8))  & 0x07FF;
	rcChannels.values[1]  = (payload[1]  >> 3 | (payload[2]  << 5))  & 0x07FF;
	rcChannels.values[2]  = (payload[2]  >> 6 | (payload[3]  << 2) | (payload[4]  << 10)) & 0x07FF;
	rcChannels.values[3]  = (payload[4]  >> 1 | (payload[5]  << 7))  & 0x07FF;
	rcChannels.values[4]  = (payload[5]  >> 4 | (payload[6]  << 4))  & 0x07FF;
	rcChannels.values[5]  = (payload[6]  >> 7 | (payload[7]  << 1) | (payload[8]  << 9)) & 0x07FF;
	rcChannels.values[6]  = (payload[8]  >> 2 | (payload[9]  << 6))  & 0x07FF;
	rcChannels.values[7]  = (payload[9]  >> 5 | (payload[10] << 3))  & 0x07FF;
	rcChannels.values[8]  = (payload[11] | (payload[12] << 8))  & 0x07FF;
	rcChannels.values[9]  = (payload[12] >> 3 | (payload[13] << 5))  & 0x07FF;
	rcChannels.values[10] = (payload[13] >> 6 | (payload[14] << 2) | (payload[15] << 10)) & 0x07FF;
	rcChannels.values[11] = (payload[15] >> 1 | (payload[16] << 7))  & 0x07FF;
	rcChannels.values[12] = (payload[16] >> 4 | (payload[17] << 4))  & 0x07FF;
	rcChannels.values[13] = (payload[17] >> 7 | (payload[18] << 1) | (payload[19] << 9)) & 0x07FF;
	rcChannels.values[14] = (payload[19] >> 2 | (payload[20] << 6))  & 0x07FF;
	rcChannels.values[15] = (payload[20] >> 5 | (payload[21] << 3))  & 0x07FF;

	rcChannels.lastUpdateTime = HAL_GetTick();
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart1, rxBuffer, RX_BUFFER_SIZE);
  /* USER CODE END 2 */

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 420000;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
