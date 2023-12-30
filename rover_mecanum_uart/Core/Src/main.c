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

#include <mecanum.h>
#include <lwrb/lwrb.h>
#include <lwpkt/lwpkt.h>
#include <stdio.h>
#include "../../cJSON-1.7.17/cJSON.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ROBOT_FL_MOTOR_TIM      htim1
#define ROBOT_FR_MOTOR_TIM      htim1
#define ROBOT_BL_MOTOR_TIM      htim2
#define ROBOT_BR_MOTOR_TIM      htim1

#define ROBOT_FL_MOTOR_TIM_CHAN TIM_CHANNEL_1
#define ROBOT_FR_MOTOR_TIM_CHAN TIM_CHANNEL_3
#define ROBOT_BL_MOTOR_TIM_CHAN TIM_CHANNEL_1
#define ROBOT_BR_MOTOR_TIM_CHAN TIM_CHANNEL_2

#define ROBOT_FL_MOTOR_DIR_PIN_1_PORT ROBOT_IN3_B_GPIO_Port
#define ROBOT_FR_MOTOR_DIR_PIN_1_PORT ROBOT_IN1_B_GPIO_Port
#define ROBOT_BL_MOTOR_DIR_PIN_1_PORT ROBOT_IN3_A_GPIO_Port
#define ROBOT_BR_MOTOR_DIR_PIN_1_PORT ROBOT_IN1_A_GPIO_Port

#define ROBOT_FL_MOTOR_DIR_PIN_2_PORT ROBOT_IN4_B_GPIO_Port
#define ROBOT_FR_MOTOR_DIR_PIN_2_PORT ROBOT_IN2_B_GPIO_Port
#define ROBOT_BL_MOTOR_DIR_PIN_2_PORT ROBOT_IN4_A_GPIO_Port
#define ROBOT_BR_MOTOR_DIR_PIN_2_PORT ROBOT_IN2_A_GPIO_Port

#define ROBOT_FL_MOTOR_DIR_PIN_1 ROBOT_IN3_B_Pin
#define ROBOT_FR_MOTOR_DIR_PIN_1 ROBOT_IN1_B_Pin
#define ROBOT_BL_MOTOR_DIR_PIN_1 ROBOT_IN3_A_Pin
#define ROBOT_BR_MOTOR_DIR_PIN_1 ROBOT_IN1_A_Pin

#define ROBOT_FL_MOTOR_DIR_PIN_2 ROBOT_IN4_B_Pin
#define ROBOT_FR_MOTOR_DIR_PIN_2 ROBOT_IN2_B_Pin
#define ROBOT_BL_MOTOR_DIR_PIN_2 ROBOT_IN4_A_Pin
#define ROBOT_BR_MOTOR_DIR_PIN_2 ROBOT_IN2_A_Pin

#define lwpkt_init_event_flag 			((uint32_t)0x00000001)

#define UART_RX_BUFFER_SIZE 512
#define UART_DMA_RX_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 512

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uart_rb_task */
osThreadId_t uart_rb_taskHandle;
const osThreadAttr_t uart_rb_task_attributes = {
  .name = "uart_rb_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for uart_rb_queue */
osMessageQueueId_t uart_rb_queueHandle;
const osMessageQueueAttr_t uart_rb_queue_attributes = {
  .name = "uart_rb_queue"
};
/* Definitions for lwpkt_events */
osEventFlagsId_t lwpkt_eventsHandle;
const osEventFlagsAttr_t lwpkt_events_attributes = {
  .name = "lwpkt_events"
};
/* USER CODE BEGIN PV */

motor_t fl_motor = { .dir_pin_1_port = ROBOT_FL_MOTOR_DIR_PIN_1_PORT,
    .dir_pin_1 =
    ROBOT_FL_MOTOR_DIR_PIN_1, .dir_pin_2_port =
    ROBOT_FL_MOTOR_DIR_PIN_2_PORT, .dir_pin_2 = ROBOT_FL_MOTOR_DIR_PIN_2,
    .timer = &ROBOT_FL_MOTOR_TIM, .channel =
    ROBOT_FL_MOTOR_TIM_CHAN, .timer_reload = 0 };

motor_t fr_motor = { .dir_pin_1_port = ROBOT_FR_MOTOR_DIR_PIN_1_PORT,
    .dir_pin_1 =
    ROBOT_FR_MOTOR_DIR_PIN_1, .dir_pin_2_port =
    ROBOT_FR_MOTOR_DIR_PIN_2_PORT, .dir_pin_2 = ROBOT_FR_MOTOR_DIR_PIN_2,
    .timer = &ROBOT_FR_MOTOR_TIM, .channel =
    ROBOT_FR_MOTOR_TIM_CHAN, .timer_reload = 0 };

motor_t bl_motor = { .dir_pin_1_port = ROBOT_BL_MOTOR_DIR_PIN_1_PORT,
    .dir_pin_1 =
    ROBOT_BL_MOTOR_DIR_PIN_1, .dir_pin_2_port =
    ROBOT_BL_MOTOR_DIR_PIN_2_PORT, .dir_pin_2 = ROBOT_BL_MOTOR_DIR_PIN_2,
    .timer = &ROBOT_BL_MOTOR_TIM, .channel =
    ROBOT_BL_MOTOR_TIM_CHAN, .timer_reload = 0 };

motor_t br_motor = { .dir_pin_1_port = ROBOT_BR_MOTOR_DIR_PIN_1_PORT,
    .dir_pin_1 =
    ROBOT_BR_MOTOR_DIR_PIN_1, .dir_pin_2_port =
    ROBOT_BR_MOTOR_DIR_PIN_2_PORT, .dir_pin_2 = ROBOT_BR_MOTOR_DIR_PIN_2,
    .timer = &ROBOT_BR_MOTOR_TIM, .channel =
    ROBOT_BR_MOTOR_TIM_CHAN, .timer_reload = 0 };

four_wheeled_robot_t robot = { .fl_motor = &fl_motor, .fr_motor = &fr_motor,
    .bl_motor = &bl_motor, .br_motor = &br_motor, };

lwpkt_t uart_lwpkt;

uint8_t uart_dma_rx_buffer[UART_DMA_RX_BUFFER_SIZE];

lwrb_t uart_rx_buffer;
uint8_t uart_rx_data_buffer[UART_RX_BUFFER_SIZE];

lwrb_t uart_tx_buffer;
uint8_t uart_tx_data_buffer[UART_TX_BUFFER_SIZE];

const cJSON_Hooks cjson_hooks = {
		.malloc_fn = pvPortMalloc,
		.free_fn = vPortFree
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void Startuart_rb_task(void *argument);

/* USER CODE BEGIN PFP */

int _write(int file, char *ptr, int len);

static void uart_lwpkt_evt_fn(lwpkt_t* pkt, lwpkt_evt_type_t type);
void uart_tx_rb_evt_fn(lwrb_t* buff, lwrb_evt_type_t type, size_t len);

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

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

  /* Create the queue(s) */
  /* creation of uart_rb_queue */
  uart_rb_queueHandle = osMessageQueueNew (10, sizeof(uint16_t), &uart_rb_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of uart_rb_task */
  uart_rb_taskHandle = osThreadNew(Startuart_rb_task, NULL, &uart_rb_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of lwpkt_events */
  lwpkt_eventsHandle = osEventFlagsNew(&lwpkt_events_attributes);

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 9;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8400;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8400;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ROBOT_IN4_A_Pin|ROBOT_IN3_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ROBOT_IN4_B_Pin|ROBOT_IN1_B_Pin|ROBOT_IN3_B_Pin|ROBOT_IN2_B_Pin
                          |ROBOT_IN2_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ROBOT_IN1_A_GPIO_Port, ROBOT_IN1_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROBOT_IN4_A_Pin ROBOT_IN3_A_Pin */
  GPIO_InitStruct.Pin = ROBOT_IN4_A_Pin|ROBOT_IN3_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ROBOT_IN4_B_Pin ROBOT_IN1_B_Pin ROBOT_IN3_B_Pin ROBOT_IN2_B_Pin
                           ROBOT_IN2_A_Pin */
  GPIO_InitStruct.Pin = ROBOT_IN4_B_Pin|ROBOT_IN1_B_Pin|ROBOT_IN3_B_Pin|ROBOT_IN2_B_Pin
                          |ROBOT_IN2_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ROBOT_IN1_A_Pin */
  GPIO_InitStruct.Pin = ROBOT_IN1_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ROBOT_IN1_A_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len){
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}

void uart_tx_rb_evt_fn(lwrb_t* buff, lwrb_evt_type_t type, size_t len){
	switch (type) {
		case LWRB_EVT_WRITE:
			lwrb_sz_t size = lwrb_get_linear_block_read_length(buff);
			HAL_UART_Transmit(&huart1, (uint8_t*)lwrb_get_linear_block_read_address(buff), size, HAL_MAX_DELAY);
			lwrb_skip(buff, size);
			size = lwrb_get_linear_block_read_length(buff);
			if (size > 0) {
					HAL_UART_Transmit(&huart1, (uint8_t*)lwrb_get_linear_block_read_address(buff), size, HAL_MAX_DELAY);
			}
			lwrb_skip(buff, size);

			break;
		default:
			break;
	}
}

static void uart_lwpkt_evt_fn(lwpkt_t* pkt, lwpkt_evt_type_t type){
	switch (type) {
		case LWPKT_EVT_PKT:
			size_t len = lwpkt_get_data_len(pkt);
			char* data = (char*)lwpkt_get_data(pkt);
			printf("Packet received, size(%d), data(%.*s)\r\n", len, len, data);

			cJSON* parsed_json = cJSON_ParseWithLength(data, len);
			if (cJSON_IsObject(parsed_json)){
				printf("A json object\r\n");

				cJSON* power_json = cJSON_GetObjectItem(parsed_json, "power");
				cJSON* theta_json = cJSON_GetObjectItem(parsed_json, "theta");
				cJSON* turn_json = cJSON_GetObjectItem(parsed_json, "turn");
				cJSON* stop_json = cJSON_GetObjectItem(parsed_json, "stop");

				if (cJSON_IsTrue(stop_json)) {
					printf("Robot stopped\r\n");
					mecanum_robot_stop(&robot);
				} else {
					if (cJSON_IsNumber(power_json) && cJSON_IsNumber(theta_json) && cJSON_IsNumber(turn_json)){
						float power = cJSON_GetNumberValue(power_json);
						float theta = cJSON_GetNumberValue(theta_json);
						float turn = cJSON_GetNumberValue(turn_json);

						printf("Power: %f, Theta: %f, Turn: %f\r\n", power, theta, turn);

						mecanum_robot_move(&robot, power, theta, turn);
					} else {
						printf("One or more key/value pairs missing\r\n");
					}
				}
			} else {
				printf("Not a json object\r\n");
			}

			cJSON_free(parsed_json);
			break;
		default:
			break;
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART1) {
    osMessageQueuePut(uart_rb_queueHandle, &Size, 0, 0);
  }
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  mecanum_robot_init(&robot);

  printf("Start\r\n");
	osEventFlagsWait(lwpkt_eventsHandle,
									lwpkt_init_event_flag,
									osFlagsNoClear | osFlagsWaitAll,
									osWaitForever);

	printf("lwpkt initialized\r\n");

	cJSON_InitHooks(&cjson_hooks);

	lwpkt_set_evt_fn(&uart_lwpkt, uart_lwpkt_evt_fn);

	/* Infinite loop */
	for(;;)
	{
		uint32_t current_time = HAL_GetTick();
		lwpkt_process(&uart_lwpkt, current_time);

		osDelay(10);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Startuart_rb_task */
/**
* @brief Function implementing the uart_rb_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startuart_rb_task */
void Startuart_rb_task(void *argument)
{
  /* USER CODE BEGIN Startuart_rb_task */
	lwrb_init(&uart_rx_buffer, uart_rx_data_buffer, UART_RX_BUFFER_SIZE);

	lwrb_init(&uart_tx_buffer, uart_tx_data_buffer, UART_TX_BUFFER_SIZE);
	lwrb_set_evt_fn(&uart_tx_buffer, uart_tx_rb_evt_fn);

	lwpkt_init(&uart_lwpkt, &uart_tx_buffer, &uart_rx_buffer);
	osEventFlagsSet(lwpkt_eventsHandle, lwpkt_init_event_flag);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_dma_rx_buffer, UART_DMA_RX_BUFFER_SIZE);

	/* Infinite loop */
	for(;;)
	{
		uint16_t Size;
		osMessageQueueGet(uart_rb_queueHandle, &Size, NULL, osWaitForever);

		static uint16_t pos = 0;
		lwrb_write(&uart_rx_buffer, &uart_dma_rx_buffer[pos], Size >= pos ? Size - pos : Size - pos + UART_DMA_RX_BUFFER_SIZE);
		pos = Size;
	}
  /* USER CODE END Startuart_rb_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
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
