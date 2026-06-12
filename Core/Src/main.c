/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "board.h"
#include "canrpc.h"
#include "robot_can.h"
#include "robot_control.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ROBOT_TEST_RACK_LIMIT_HOLD_MS  1000u
#define ROBOT_START_CANRPC_TIMEOUT_MS  1000u
#define ROBOT_POSE_PRINT_PERIOD_MS     1000u
#define ROBOT_SENSOR_CANRPC_TIMEOUT_MS 1000u
#define ROBOT_SENSOR_TEST_PERIOD_MS    1000u

#define TOPIC_POSE2D             0x10u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

FDCAN_HandleTypeDef hfdcan1;

FMAC_HandleTypeDef hfmac;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
typedef struct {
  bool valid;
  uint8_t seq;
  uint32_t sensor_t_ms;
  uint32_t rx_t_ms;
  int32_t x_mm;
  int32_t y_mm;
  float h_rad;
  uint16_t status_flags;
} Robot_Pose2D;

static volatile Robot_Pose2D g_sensor_pose;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FMAC_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
static void Robot_SendStartCommandToRemoteNodes(void);
static void Robot_PrintPose(uint32_t now_ms);
static void Robot_PrintSensorSample(const RobotControl_SensorSample *sample);
static void Robot_OnCanrpcPublish(uint8_t node, uint8_t topic, const uint8_t *data, uint8_t len);
static bool Robot_TimeReached(uint32_t now_ms, uint32_t deadline_ms);
static Robot_Pose2D Robot_GetPoseSnapshot(void);
static float Robot_PoseHeadingToNodeRad(int32_t h_mrad_ccw);
static int32_t Robot_RadToMradForPrint(float rad);
static uint32_t Robot_AbsI32ToU32(int32_t value);
static uint16_t Robot_GetU16Le(const uint8_t *p);
static uint32_t Robot_GetU32Le(const uint8_t *p);
static int32_t Robot_GetI32Le(const uint8_t *p);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&hlpuart1,(uint8_t *)ptr,len,10);
  return len;
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
  MX_FDCAN1_Init();
  MX_FMAC_Init();
  MX_DAC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_LPUART1_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  Board_Init();
  Board_StepperStopAll();
  Board_DCMotorSwingStop();
  Board_DCMotorRackStop();
  canrpc_init(NODE_MASTER);
  if (!canrpc_start(NODE_MASTER)) {
    Error_Handler();
  }
  canrpc_set_pub_handler(Robot_OnCanrpcPublish);
  RobotControl_Init();
  if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) {
    Error_Handler();
  }
  Board_ClearStartSwitchInterruptStatus();
  Board_WaitForStartSwitchInterrupt();
  Robot_SendStartCommandToRemoteNodes();
  Board_StepperStopAll();
  uint32_t next_pose_print_ms = HAL_GetTick() + ROBOT_POSE_PRINT_PERIOD_MS;
  uint32_t next_sensor_test_ms = HAL_GetTick() + ROBOT_SENSOR_TEST_PERIOD_MS;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now_ms = HAL_GetTick();
    if (Robot_TimeReached(now_ms, next_pose_print_ms)) {
      next_pose_print_ms = now_ms + ROBOT_POSE_PRINT_PERIOD_MS;
      Robot_PrintPose(now_ms);
    }

    if (Robot_TimeReached(now_ms, next_sensor_test_ms)) {
      RobotControl_SensorSample sample;
      (void)RobotControl_ReadSensorSampleBlocking(&sample, ROBOT_SENSOR_CANRPC_TIMEOUT_MS);
      Robot_PrintSensorSample(&sample);
      next_sensor_test_ms = HAL_GetTick() + ROBOT_SENSOR_TEST_PERIOD_MS;
    }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 5;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FMAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMAC_Init(void)
{

  /* USER CODE BEGIN FMAC_Init 0 */

  /* USER CODE END FMAC_Init 0 */

  /* USER CODE BEGIN FMAC_Init 1 */

  /* USER CODE END FMAC_Init 1 */
  hfmac.Instance = FMAC;
  if (HAL_FMAC_Init(&hfmac) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMAC_Init 2 */

  /* USER CODE END FMAC_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 79;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 199;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DCMotorSwing_DIR_Pin|DCMotorRack_DIR_Pin|DIR1_Pin|DIR2_Pin
                          |ENABLE_Pin|RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DCMotorSwing_DIR_Pin DCMotorRack_DIR_Pin DIR1_Pin DIR2_Pin
                           ENABLE_Pin RESET_Pin */
  GPIO_InitStruct.Pin = DCMotorSwing_DIR_Pin|DCMotorRack_DIR_Pin|DIR1_Pin|DIR2_Pin
                          |ENABLE_Pin|RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LimitSW_Swing_Pin LimitSW_Rack_Close_Pin START_SW_Pin USER_SW_Pin
                           LimitSW_Rack_Open_Pin */
  GPIO_InitStruct.Pin = LimitSW_Swing_Pin|LimitSW_Rack_Close_Pin|START_SW_Pin|USER_SW_Pin
                          |LimitSW_Rack_Open_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FAULT_Pin */
  GPIO_InitStruct.Pin = FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void Robot_SendStartCommandToRemoteNodes(void)
{
  int sensor_handle = canrpc_call(NODE_SENSOR, CMD_SYSTEM_START, 0);
  int servo_handle = canrpc_call(NODE_SERVO, CMD_SYSTEM_START, 0);

  if ((sensor_handle < 0) || (servo_handle < 0)) {
    Error_Handler();
  }

  uint32_t wait_mask = CANRPC_H(sensor_handle) | CANRPC_H(servo_handle);
  int rc = canrpc_wait(wait_mask, ROBOT_START_CANRPC_TIMEOUT_MS);
  if (rc != 0) {
    volatile uint8_t sensor_result = canrpc_result(sensor_handle);
    volatile uint8_t servo_result = canrpc_result(servo_handle);
    (void)sensor_result;
    (void)servo_result;
    Error_Handler();
  }
}

static void Robot_PrintPose(uint32_t now_ms)
{
  Robot_Pose2D pose = Robot_GetPoseSnapshot();
  uint32_t pose_age_ms = pose.valid ? (uint32_t)(now_ms - pose.rx_t_ms) : 0u;
  int32_t h_mrad_for_print = Robot_RadToMradForPrint(pose.h_rad);
  uint32_t h_abs_mrad = Robot_AbsI32ToU32(h_mrad_for_print);

  printf("pose[valid=%u seq=%u sensor_t=%lums age=%lums status=0x%04x x=%ld y=%ld h=%s%lu.%03lurad]\r\n",
         pose.valid ? 1u : 0u,
         (unsigned int)pose.seq,
         (unsigned long)pose.sensor_t_ms,
         (unsigned long)pose_age_ms,
         (unsigned int)pose.status_flags,
         (long)pose.x_mm,
         (long)pose.y_mm,
         h_mrad_for_print < 0 ? "-" : "",
         (unsigned long)(h_abs_mrad / 1000u),
         (unsigned long)(h_abs_mrad % 1000u));
}

static void Robot_PrintSensorSample(const RobotControl_SensorSample *sample)
{
  const RobotControl_Tsd10 *tsd = &sample->tsd;
  const RobotControl_ColorRaw *color = &sample->color;
  uint32_t now_ms = HAL_GetTick();
  uint32_t color_age_ms = color->valid ? (uint32_t)(now_ms - color->rx_t_ms) : 0u;

  printf("tsd10[status=%d ch0=%u(valid=%u res=0x%02x) ch1=%u(valid=%u res=0x%02x) ch2=%u(valid=%u res=0x%02x)]\r\n",
         sample->tsd_status,
         (unsigned int)tsd->distance_mm[0],
         tsd->valid[0] ? 1u : 0u,
         (unsigned int)tsd->rpc_result[0],
         (unsigned int)tsd->distance_mm[1],
         tsd->valid[1] ? 1u : 0u,
         (unsigned int)tsd->rpc_result[1],
         (unsigned int)tsd->distance_mm[2],
         tsd->valid[2] ? 1u : 0u,
         (unsigned int)tsd->rpc_result[2]);

  printf("color[status=%d valid=%u res=0x%02x seq=%u sensor_t=%lums age=%lums c=%u r=%u g=%u b=%u atime=0x%02x gain=%u led=%ums flags=0x%02x]\r\n",
         sample->color_status,
         color->valid ? 1u : 0u,
         (unsigned int)color->rpc_result,
         (unsigned int)color->seq,
         (unsigned long)color->sensor_t_ms,
         (unsigned long)color_age_ms,
         (unsigned int)color->clear,
         (unsigned int)color->red,
         (unsigned int)color->green,
         (unsigned int)color->blue,
         (unsigned int)color->atime,
         (unsigned int)color->gain,
         (unsigned int)color->led_on_ms,
         (unsigned int)color->flags);
}

static void Robot_OnCanrpcPublish(uint8_t node, uint8_t topic, const uint8_t *data, uint8_t len)
{
  RobotControl_OnCanrpcPublish(node, topic, data, len);

  if (node != NODE_SENSOR || topic != TOPIC_POSE2D || data == NULL || len < 19u) {
    return;
  }

  g_sensor_pose.seq = data[0];
  g_sensor_pose.sensor_t_ms = Robot_GetU32Le(&data[1]);
  g_sensor_pose.x_mm = Robot_GetI32Le(&data[5]);
  g_sensor_pose.y_mm = Robot_GetI32Le(&data[9]);
  g_sensor_pose.h_rad = Robot_PoseHeadingToNodeRad(Robot_GetI32Le(&data[13]));
  g_sensor_pose.status_flags = Robot_GetU16Le(&data[17]);
  g_sensor_pose.rx_t_ms = HAL_GetTick();
  g_sensor_pose.valid = true;
}

static bool Robot_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
  return (int32_t)(now_ms - deadline_ms) >= 0;
}

static Robot_Pose2D Robot_GetPoseSnapshot(void)
{
  Robot_Pose2D pose;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  pose.valid = g_sensor_pose.valid;
  pose.seq = g_sensor_pose.seq;
  pose.sensor_t_ms = g_sensor_pose.sensor_t_ms;
  pose.rx_t_ms = g_sensor_pose.rx_t_ms;
  pose.x_mm = g_sensor_pose.x_mm;
  pose.y_mm = g_sensor_pose.y_mm;
  pose.h_rad = g_sensor_pose.h_rad;
  pose.status_flags = g_sensor_pose.status_flags;
  if (primask == 0u) {
    __enable_irq();
  }

  return pose;
}

static float Robot_PoseHeadingToNodeRad(int32_t h_mrad_ccw)
{
  return -(float)h_mrad_ccw * 0.001f;
}

static int32_t Robot_RadToMradForPrint(float rad)
{
  float h_mrad = rad * 1000.0f;
  return (int32_t)(h_mrad >= 0.0f ? h_mrad + 0.5f : h_mrad - 0.5f);
}

static uint32_t Robot_AbsI32ToU32(int32_t value)
{
  return (value < 0) ? (uint32_t)(-(value + 1)) + 1u : (uint32_t)value;
}

static uint16_t Robot_GetU16Le(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t Robot_GetU32Le(const uint8_t *p)
{
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static int32_t Robot_GetI32Le(const uint8_t *p)
{
  return (int32_t)Robot_GetU32Le(p);
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
