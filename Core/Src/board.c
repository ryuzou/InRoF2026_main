#include "board.h"

#include "canrpc.h"
#include "stdio.h"

#include <string.h>

#define BOARD_STEPPER_TIM_HZ       1000000u  /* TIM3: 80 MHz / (79 + 1) */
#define BOARD_VDDA_MV              3300u
#define BOARD_STEPPER_CURRENT_MA   1000u
#define BOARD_MAX_WHEEL_SPEED_MM_S  500u
#define BOARD_STEPPER_AXIS_COUNT   2u
#define BOARD_DCMOTOR_DUTY_SCALE   1000u

#ifndef BOARD_DCMOTOR_SWING_DIR0_DUTY_PERMILLE
#define BOARD_DCMOTOR_SWING_DIR0_DUTY_PERMILLE  500u
#endif

#ifndef BOARD_DCMOTOR_SWING_DIR1_DUTY_PERMILLE
#define BOARD_DCMOTOR_SWING_DIR1_DUTY_PERMILLE  500u
#endif

#ifndef BOARD_DCMOTOR_RACK_DIR0_DUTY_PERMILLE
#define BOARD_DCMOTOR_RACK_DIR0_DUTY_PERMILLE  500u
#endif

#ifndef BOARD_DCMOTOR_RACK_DIR1_DUTY_PERMILLE
#define BOARD_DCMOTOR_RACK_DIR1_DUTY_PERMILLE  500u
#endif

typedef struct {
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  GPIO_PinState forward_pin_state;
  volatile uint16_t half_ticks;
  volatile uint8_t running;
} BoardStepperAxis;

extern FDCAN_HandleTypeDef hfdcan1;
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

static BoardStepperAxis s_stepper[BOARD_STEPPER_AXIS_COUNT] = {
  { &htim3, TIM_CHANNEL_3, DIR2_GPIO_Port, DIR2_Pin, BOARD_STEPPER_LEFT_FORWARD_PIN_STATE, 0u, 0u },
  { &htim3, TIM_CHANNEL_1, DIR1_GPIO_Port, DIR1_Pin, BOARD_STEPPER_RIGHT_FORWARD_PIN_STATE, 0u, 0u },
};

static volatile bool s_fault_interrupt_seen = false;
static volatile uint32_t s_fault_interrupt_count = 0u;
static volatile bool s_limit_switch_swing_interrupt_seen = false;
static volatile uint32_t s_limit_switch_swing_interrupt_count = 0u;
static volatile bool s_limit_switch_rack_open_interrupt_seen = false;
static volatile uint32_t s_limit_switch_rack_open_interrupt_count = 0u;
static volatile bool s_limit_switch_rack_close_interrupt_seen = false;
static volatile uint32_t s_limit_switch_rack_close_interrupt_count = 0u;
static volatile uint8_t s_dcmotor_swing_running = 0u;
static volatile uint8_t s_dcmotor_rack_running = 0u;
static volatile bool s_start_switch_interrupt_seen = false;
static volatile uint32_t s_start_switch_interrupt_count = 0u;
static volatile bool s_user_switch_interrupt_seen = false;
static volatile uint32_t s_user_switch_interrupt_count = 0u;
static bool s_canrpc_started = false;

static uint32_t TB67_DacCodeFromCurrent_mA(uint32_t current_mA, uint32_t vdda_mV);
static uint32_t Board_DCMotorDutyToPulse(TIM_HandleTypeDef *htim, uint32_t duty_permille);
static uint32_t Board_FDCANLenToDlc(uint8_t len);
static uint8_t Board_FDCANDlcToLen(uint32_t dlc);
static bool Board_DCMotorSwingDirectionIsValid(BoardDCMotorSwingDirection direction);
static uint32_t Board_DCMotorSwingDutyPermille(BoardDCMotorSwingDirection direction);
static bool Board_DCMotorRackDirectionIsValid(BoardDCMotorRackDirection direction);
static uint32_t Board_DCMotorRackDutyPermille(BoardDCMotorRackDirection direction);
static bool Board_LimitSwitchSwingIsPressed(void);
static bool Board_LimitSwitchRackOpenIsPressed(void);
static bool Board_LimitSwitchRackCloseIsPressed(void);
static bool Board_UserSwitchIsPressed(void);
static void Board_ClearUserSwitchInterruptStatus(void);
static void Board_UpdateSwingLimitLed(void);
static void Board_DCMotorSwingWaitForLimitRelease(void);
static void Board_DCMotorRackWaitForOpenLimitRelease(void);
static void Board_DCMotorRackWaitForCloseLimitRelease(void);
static void Board_StepperSetCurrent_mA(uint32_t current_mA);
static void Board_StepperEnable(bool enable);
static void Board_StepperResetPulse(void);
static void Board_StepperScheduleNext(BoardStepperAxis *axis);
static uint32_t Board_StepperChannelToFlag(uint32_t channel);
static bool Board_StepperMotorIsValid(BoardStepperMotor motor);

void Board_Init(void)
{
  Board_StepperEnable(false);
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);

  if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }

  Board_StepperSetCurrent_mA(BOARD_STEPPER_CURRENT_MA);

  /*
   * Give DAC output and motor power rails a short settling time.
   * In a product sequence, VM detection should replace this blind delay.
   */
  HAL_Delay(10);

  Board_StepperResetPulse();

  __HAL_TIM_SET_COUNTER(&htim3, 0u);
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1 | TIM_FLAG_CC3 | TIM_FLAG_UPDATE);

  Board_ClearFaultInterruptStatus();
  Board_StepperEnable(true);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
}

uint32_t board_millis(void)
{
  return HAL_GetTick();
}

void board_idle(void)
{
  __WFI();
}

uint32_t Board_MaxWheelSpeed_mm_s(void)
{
  return BOARD_MAX_WHEEL_SPEED_MM_S;
}

void Board_StepperStart(
    BoardStepperMotor motor,
    BoardStepperDirection direction,
    uint32_t step_hz
)
{
  if (!Board_StepperMotorIsValid(motor)) {
    return;
  }

  if (step_hz == 0u) {
    Board_StepperStop(motor);
    return;
  }

  BoardStepperAxis *axis = &s_stepper[(uint32_t)motor];

  if (direction == BOARD_STEPPER_DIR_FORWARD) {
    HAL_GPIO_WritePin(axis->dir_port, axis->dir_pin, axis->forward_pin_state);
  } else {
    /*
     * Do not toggle unconditionally; repeated REVERSE commands must keep the
     * same physical direction.
     */
    if (HAL_GPIO_ReadPin(axis->dir_port, axis->dir_pin) == axis->forward_pin_state) {
      HAL_GPIO_TogglePin(axis->dir_port, axis->dir_pin);
    }
  }

  /*
   * Output Compare Toggle toggles every half period.
   * Rising edges only are counted as STEP by the driver.
   */
  uint32_t half_ticks = BOARD_STEPPER_TIM_HZ / (2u * step_hz);

  if (half_ticks < 2u) {
    half_ticks = 2u;
  }

  if (half_ticks > 0xFFFFu) {
    half_ticks = 0xFFFFu;
  }

  axis->half_ticks = (uint16_t)half_ticks;

  if (axis->running == 0u) {
    uint32_t now = __HAL_TIM_GET_COUNTER(axis->htim);
    uint32_t first_ccr = (now + axis->half_ticks) & 0xFFFFu;

    __HAL_TIM_SET_COMPARE(axis->htim, axis->channel, first_ccr);

    uint32_t flag = Board_StepperChannelToFlag(axis->channel);
    if (flag != 0u) {
      __HAL_TIM_CLEAR_FLAG(axis->htim, flag);
    }

    axis->running = 1u;

    if (HAL_TIM_OC_Start_IT(axis->htim, axis->channel) != HAL_OK) {
      Error_Handler();
    }
  }
}

void Board_StepperStop(BoardStepperMotor motor)
{
  if (!Board_StepperMotorIsValid(motor)) {
    return;
  }

  BoardStepperAxis *axis = &s_stepper[(uint32_t)motor];

  axis->running = 0u;
  axis->half_ticks = 0u;

  (void)HAL_TIM_OC_Stop_IT(axis->htim, axis->channel);
}

void Board_StepperStopAll(void)
{
  Board_StepperStop(BOARD_STEPPER_LEFT);
  Board_StepperStop(BOARD_STEPPER_RIGHT);
}

void Board_DCMotorSwingStart(BoardDCMotorSwingDirection direction)
{
  if (!Board_DCMotorSwingDirectionIsValid(direction)) {
    return;
  }

  HAL_GPIO_WritePin(
      DCMotorSwing_DIR_GPIO_Port,
      DCMotorSwing_DIR_Pin,
      (direction == BOARD_DCMOTOR_SWING_DIR_0) ? GPIO_PIN_RESET : GPIO_PIN_SET
  );

  uint32_t pulse = Board_DCMotorDutyToPulse(
      &htim1,
      Board_DCMotorSwingDutyPermille(direction)
  );

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);

  if (s_dcmotor_swing_running == 0u) {
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
      Error_Handler();
    }

    s_dcmotor_swing_running = 1u;
  }
}

void Board_DCMotorSwingStop(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
  HAL_GPIO_WritePin(
    DCMotorSwing_DIR_GPIO_Port,
    DCMotorSwing_DIR_Pin,
    GPIO_PIN_RESET
  );

  if (s_dcmotor_swing_running != 0u) {
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    s_dcmotor_swing_running = 0u;
  }
}

void Board_DCMotorSwingMoveUntilLimit(BoardDCMotorSwingDirection direction)
{
  if (!Board_DCMotorSwingDirectionIsValid(direction)) {
    return;
  }

  bool limit_was_pressed = Board_LimitSwitchSwingIsPressed();

  Board_ClearLimitSwitchSwingInterruptStatus();
  Board_DCMotorSwingStart(direction);

  if (limit_was_pressed) {
    Board_DCMotorSwingWaitForLimitRelease();
    Board_ClearLimitSwitchSwingInterruptStatus();
  }

  Board_WaitForLimitSwitchSwingInterrupt();
  Board_DCMotorSwingStop();
}

void Board_DCMotorSwingRaiseUntilLimit(void)
{
  Board_DCMotorSwingMoveUntilLimit(BOARD_DCMOTOR_SWING_RAISE_DIRECTION);
}

void Board_DCMotorSwingLowerUntilLimit(void)
{
  Board_DCMotorSwingMoveUntilLimit(BOARD_DCMOTOR_SWING_LOWER_DIRECTION);
}

void Board_DCMotorRackStart(BoardDCMotorRackDirection direction)
{
  if (!Board_DCMotorRackDirectionIsValid(direction)) {
    return;
  }

  HAL_GPIO_WritePin(
      DCMotorRack_DIR_GPIO_Port,
      DCMotorRack_DIR_Pin,
      (direction == BOARD_DCMOTOR_RACK_DIR_0) ? GPIO_PIN_RESET : GPIO_PIN_SET
  );

  uint32_t pulse = Board_DCMotorDutyToPulse(
      &htim1,
      Board_DCMotorRackDutyPermille(direction)
  );

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);

  if (s_dcmotor_rack_running == 0u) {
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
      Error_Handler();
    }

    s_dcmotor_rack_running = 1u;
  }
}

void Board_DCMotorRackStop(void)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
  HAL_GPIO_WritePin(
    DCMotorRack_DIR_GPIO_Port,
    DCMotorRack_DIR_Pin,
    GPIO_PIN_RESET
  );

  if (s_dcmotor_rack_running != 0u) {
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    s_dcmotor_rack_running = 0u;
  }
}

void Board_DCMotorRackOpenUntilLimit(void)
{
  bool limit_was_pressed = Board_LimitSwitchRackOpenIsPressed();

  Board_ClearLimitSwitchRackOpenInterruptStatus();

  if (!limit_was_pressed) {
    Board_DCMotorRackStart(BOARD_DCMOTOR_RACK_OPEN_DIRECTION);
    Board_WaitForLimitSwitchRackOpenInterrupt();
  }
  Board_DCMotorRackStop();
}

void Board_DCMotorRackCloseUntilLimit(void)
{
  bool limit_was_pressed = Board_LimitSwitchRackCloseIsPressed();

  Board_ClearLimitSwitchRackCloseInterruptStatus();

  if (!limit_was_pressed) {
    Board_DCMotorRackStart(BOARD_DCMOTOR_RACK_CLOSE_DIRECTION);
    Board_WaitForLimitSwitchRackCloseInterrupt();
  }
  Board_DCMotorRackStop();
}

bool Board_FaultInterruptSeen(void)
{
  return s_fault_interrupt_seen;
}

uint32_t Board_FaultInterruptCount(void)
{
  return s_fault_interrupt_count;
}

void Board_ClearFaultInterruptStatus(void)
{
  s_fault_interrupt_seen = false;
  s_fault_interrupt_count = 0u;
}

bool Board_LimitSwitchSwingInterruptSeen(void)
{
  return s_limit_switch_swing_interrupt_seen;
}

uint32_t Board_LimitSwitchSwingInterruptCount(void)
{
  return s_limit_switch_swing_interrupt_count;
}

void Board_ClearLimitSwitchSwingInterruptStatus(void)
{
  s_limit_switch_swing_interrupt_seen = false;
  s_limit_switch_swing_interrupt_count = 0u;
}

void Board_WaitForLimitSwitchSwingInterrupt(void)
{
  while (!Board_LimitSwitchSwingInterruptSeen()) {
    __WFI();
  }
}

bool Board_LimitSwitchRackOpenInterruptSeen(void)
{
  return s_limit_switch_rack_open_interrupt_seen;
}

uint32_t Board_LimitSwitchRackOpenInterruptCount(void)
{
  return s_limit_switch_rack_open_interrupt_count;
}

void Board_ClearLimitSwitchRackOpenInterruptStatus(void)
{
  s_limit_switch_rack_open_interrupt_seen = false;
  s_limit_switch_rack_open_interrupt_count = 0u;
}

void Board_WaitForLimitSwitchRackOpenInterrupt(void)
{
  while (!Board_LimitSwitchRackOpenInterruptSeen()) {
    __WFI();
  }
}

bool Board_LimitSwitchRackCloseInterruptSeen(void)
{
  return s_limit_switch_rack_close_interrupt_seen;
}

uint32_t Board_LimitSwitchRackCloseInterruptCount(void)
{
  return s_limit_switch_rack_close_interrupt_count;
}

void Board_ClearLimitSwitchRackCloseInterruptStatus(void)
{
  s_limit_switch_rack_close_interrupt_seen = false;
  s_limit_switch_rack_close_interrupt_count = 0u;
}

void Board_WaitForLimitSwitchRackCloseInterrupt(void)
{
  while (!Board_LimitSwitchRackCloseInterruptSeen()) {
    __WFI();
  }
}

bool Board_StartSwitchInterruptSeen(void)
{
  return s_start_switch_interrupt_seen;
}

uint32_t Board_StartSwitchInterruptCount(void)
{
  return s_start_switch_interrupt_count;
}

bool board_canrpc_start(uint8_t own_addr)
{
  if (s_canrpc_started) {
    return true;
  }

  FDCAN_FilterTypeDef filter = {
    .IdType = FDCAN_STANDARD_ID,
    .FilterType = FDCAN_FILTER_MASK,
    .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
    .FilterID1 = CANRPC_ID(CANRPC_TYPE_ACK, 0),
    .FilterID2 = 0x700u,
  };

  filter.FilterIndex = 0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
    return false;
  }

  filter.FilterIndex = 1;
  filter.FilterID1 = CANRPC_ID(CANRPC_TYPE_DONE, 0);
  filter.FilterID2 = 0x700u;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
    return false;
  }

  filter.FilterIndex = 2;
  filter.FilterID1 = CANRPC_ID(CANRPC_TYPE_CMD, own_addr);
  filter.FilterID2 = 0x7FFu;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
    return false;
  }

  filter.FilterIndex = 3;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  filter.FilterID1 = CANRPC_ID(CANRPC_TYPE_PUB, 0);
  filter.FilterID2 = 0x700u;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
    return false;
  }

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
    return false;
  }

  if (HAL_FDCAN_ConfigInterruptLines(&hfdcan1,
                                     FDCAN_IT_GROUP_RX_FIFO0,
                                     FDCAN_INTERRUPT_LINE0) != HAL_OK) {
    return false;
  }

  if (HAL_FDCAN_ConfigInterruptLines(&hfdcan1,
                                     FDCAN_IT_GROUP_RX_FIFO1 | FDCAN_IT_GROUP_BIT_LINE_ERROR,
                                     FDCAN_INTERRUPT_LINE1) != HAL_OK) {
    return false;
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
    return false;
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
    return false;
  }

  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    return false;
  }

  s_canrpc_started = true;
  return true;
}

bool board_canrpc_tx(uint16_t id, const uint8_t *data, uint8_t len)
{
  if (!s_canrpc_started || len > CANRPC_FRAME_MAX_LEN) {
    return false;
  }

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0u) {
    return false;
  }

  uint8_t txbuf[CANRPC_FRAME_MAX_LEN] = {0};
  if (len > 0u && data != NULL) {
    memcpy(txbuf, data, len);
  }

  FDCAN_TxHeaderTypeDef header = {
    .Identifier = id,
    .IdType = FDCAN_STANDARD_ID,
    .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = Board_FDCANLenToDlc(len),
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch = FDCAN_BRS_ON,
    .FDFormat = FDCAN_FD_CAN,
    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    .MessageMarker = 0,
  };

  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, txbuf) == HAL_OK;
}

void Board_ClearStartSwitchInterruptStatus(void)
{
  s_start_switch_interrupt_seen = false;
  s_start_switch_interrupt_count = 0u;
}

void Board_WaitForStartSwitchInterrupt(void)
{
  Board_ClearUserSwitchInterruptStatus();

  while (!Board_StartSwitchInterruptSeen()) {
    Board_UpdateSwingLimitLed();

    if (Board_StartSwitchInterruptSeen()) {
      break;
    }

    if (Board_UserSwitchIsPressed()) {
      Board_ClearUserSwitchInterruptStatus();
      Board_DCMotorSwingRaiseUntilLimit();
      Board_ClearUserSwitchInterruptStatus();

      continue;
    }

    __WFI();
  }

  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
}

__weak void Board_LimitSwitchSwingInterruptHook(void)
{
  /*
   * Override this function elsewhere if the swing limit switch needs
   * immediate ISR-side work.
   */
}

__weak void Board_LimitSwitchRackOpenInterruptHook(void)
{
  /*
   * Override this function elsewhere if the rack open limit switch needs
   * immediate ISR-side work.
   */
}

__weak void Board_LimitSwitchRackCloseInterruptHook(void)
{
  /*
   * Override this function elsewhere if the rack close limit switch needs
   * immediate ISR-side work.
   */
}

__weak void Board_StartSwitchInterruptHook(void)
{
  /*
   * Override this function elsewhere if START_SW needs immediate ISR-side work.
   * The default behavior only releases Board_WaitForStartSwitchInterrupt().
   */
}

__weak void Board_FaultInterruptHook(void)
{
  /*
   * Add optional FAULT handling here or override this function elsewhere.
   * Motor control intentionally does not stop from the FAULT interrupt path.
   */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == LimitSW_Swing_Pin) {
    s_limit_switch_swing_interrupt_seen = true;
    s_limit_switch_swing_interrupt_count++;
    Board_LimitSwitchSwingInterruptHook();
  } else if (GPIO_Pin == LimitSW_Rack_Open_Pin) {
    s_limit_switch_rack_open_interrupt_seen = true;
    s_limit_switch_rack_open_interrupt_count++;
    Board_LimitSwitchRackOpenInterruptHook();
  } else if (GPIO_Pin == LimitSW_Rack_Close_Pin) {
    s_limit_switch_rack_close_interrupt_seen = true;
    s_limit_switch_rack_close_interrupt_count++;
    Board_LimitSwitchRackCloseInterruptHook();
  } else if (GPIO_Pin == START_SW_Pin) {
    s_start_switch_interrupt_seen = true;
    s_start_switch_interrupt_count++;
    Board_StartSwitchInterruptHook();
  } else if (GPIO_Pin == USER_SW_Pin) {
    s_user_switch_interrupt_seen = true;
    s_user_switch_interrupt_count++;
  } else if (GPIO_Pin == FAULT_Pin) {
    s_fault_interrupt_seen = true;
    s_fault_interrupt_count++;
    Board_FaultInterruptHook();
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u) {
    FDCAN_RxHeaderTypeDef header;
    uint8_t payload[CANRPC_FRAME_MAX_LEN] = {0};
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, payload) != HAL_OK) {
      break;
    }

    if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME) {
      continue;
    }

    uint8_t len = Board_FDCANDlcToLen(header.DataLength);
    if (len <= CANRPC_FRAME_MAX_LEN) {
      canrpc_on_rx((uint16_t)header.Identifier, payload, len);
    }
  }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0u) {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0u) {
    FDCAN_RxHeaderTypeDef header;
    uint8_t payload[CANRPC_FRAME_MAX_LEN] = {0};

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &header, payload) != HAL_OK) {
      break;
    }

    if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME) {
      continue;
    }

    uint8_t len = Board_FDCANDlcToLen(header.DataLength);
    if (len <= CANRPC_FRAME_MAX_LEN) {
      canrpc_on_rx((uint16_t)header.Identifier, payload, len);
    }
  }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM3) {
    return;
  }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    Board_StepperScheduleNext(&s_stepper[BOARD_STEPPER_RIGHT]);
  } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
    Board_StepperScheduleNext(&s_stepper[BOARD_STEPPER_LEFT]);
  }
}

static uint32_t TB67_DacCodeFromCurrent_mA(uint32_t current_mA, uint32_t vdda_mV)
{
  /*
   * TB67S249FTG current limit:
   *   IOUT[A] = VREF[V] * 1.25
   * Therefore VREF[mV] = current_mA * 4 / 5.
   */
  uint32_t vref_mV = (current_mA * 4u + 2u) / 5u;

  if (vref_mV > vdda_mV) {
    vref_mV = vdda_mV;
  }

  return (uint32_t)(((uint64_t)vref_mV * 4095u + vdda_mV / 2u) / vdda_mV);
}

static uint32_t Board_DCMotorDutyToPulse(TIM_HandleTypeDef *htim, uint32_t duty_permille)
{
  if (duty_permille > BOARD_DCMOTOR_DUTY_SCALE) {
    duty_permille = BOARD_DCMOTOR_DUTY_SCALE;
  }

  uint32_t period_ticks = __HAL_TIM_GET_AUTORELOAD(htim) + 1u;

  return (uint32_t)(((uint64_t)period_ticks * duty_permille
      + BOARD_DCMOTOR_DUTY_SCALE / 2u) / BOARD_DCMOTOR_DUTY_SCALE);
}

static uint32_t Board_FDCANLenToDlc(uint8_t len)
{
  if (len == 0u) {
    return FDCAN_DLC_BYTES_0;
  }
  if (len <= 1u) {
    return FDCAN_DLC_BYTES_1;
  }
  if (len <= 2u) {
    return FDCAN_DLC_BYTES_2;
  }
  if (len <= 3u) {
    return FDCAN_DLC_BYTES_3;
  }
  if (len <= 4u) {
    return FDCAN_DLC_BYTES_4;
  }
  if (len <= 5u) {
    return FDCAN_DLC_BYTES_5;
  }
  if (len <= 6u) {
    return FDCAN_DLC_BYTES_6;
  }
  if (len <= 7u) {
    return FDCAN_DLC_BYTES_7;
  }
  if (len <= 8u) {
    return FDCAN_DLC_BYTES_8;
  }
  if (len <= 12u) {
    return FDCAN_DLC_BYTES_12;
  }
  if (len <= 16u) {
    return FDCAN_DLC_BYTES_16;
  }
  if (len <= 20u) {
    return FDCAN_DLC_BYTES_20;
  }
  if (len <= 24u) {
    return FDCAN_DLC_BYTES_24;
  }
  if (len <= 32u) {
    return FDCAN_DLC_BYTES_32;
  }
  if (len <= 48u) {
    return FDCAN_DLC_BYTES_48;
  }

  return FDCAN_DLC_BYTES_64;
}

static uint8_t Board_FDCANDlcToLen(uint32_t dlc)
{
  switch (dlc) {
    case FDCAN_DLC_BYTES_0:
      return 0u;
    case FDCAN_DLC_BYTES_1:
      return 1u;
    case FDCAN_DLC_BYTES_2:
      return 2u;
    case FDCAN_DLC_BYTES_3:
      return 3u;
    case FDCAN_DLC_BYTES_4:
      return 4u;
    case FDCAN_DLC_BYTES_5:
      return 5u;
    case FDCAN_DLC_BYTES_6:
      return 6u;
    case FDCAN_DLC_BYTES_7:
      return 7u;
    case FDCAN_DLC_BYTES_8:
      return 8u;
    case FDCAN_DLC_BYTES_12:
      return 12u;
    case FDCAN_DLC_BYTES_16:
      return 16u;
    case FDCAN_DLC_BYTES_20:
      return 20u;
    case FDCAN_DLC_BYTES_24:
      return 24u;
    case FDCAN_DLC_BYTES_32:
      return 32u;
    case FDCAN_DLC_BYTES_48:
      return 48u;
    case FDCAN_DLC_BYTES_64:
      return 64u;
    default:
      return 0u;
  }
}

static bool Board_DCMotorSwingDirectionIsValid(BoardDCMotorSwingDirection direction)
{
  return (direction == BOARD_DCMOTOR_SWING_DIR_0)
      || (direction == BOARD_DCMOTOR_SWING_DIR_1);
}

static uint32_t Board_DCMotorSwingDutyPermille(BoardDCMotorSwingDirection direction)
{
  if (direction == BOARD_DCMOTOR_SWING_DIR_0) {
    return BOARD_DCMOTOR_SWING_DIR0_DUTY_PERMILLE;
  }

  return BOARD_DCMOTOR_SWING_DIR1_DUTY_PERMILLE;
}

static bool Board_DCMotorRackDirectionIsValid(BoardDCMotorRackDirection direction)
{
  return (direction == BOARD_DCMOTOR_RACK_DIR_0)
      || (direction == BOARD_DCMOTOR_RACK_DIR_1);
}

static uint32_t Board_DCMotorRackDutyPermille(BoardDCMotorRackDirection direction)
{
  if (direction == BOARD_DCMOTOR_RACK_DIR_0) {
    return BOARD_DCMOTOR_RACK_DIR0_DUTY_PERMILLE;
  }

  return BOARD_DCMOTOR_RACK_DIR1_DUTY_PERMILLE;
}

static bool Board_LimitSwitchSwingIsPressed(void)
{
  return HAL_GPIO_ReadPin(LimitSW_Swing_GPIO_Port, LimitSW_Swing_Pin) == GPIO_PIN_RESET;
}

static bool Board_LimitSwitchRackOpenIsPressed(void)
{
  return HAL_GPIO_ReadPin(LimitSW_Rack_Open_GPIO_Port, LimitSW_Rack_Open_Pin)
      == GPIO_PIN_RESET;
}

static bool Board_LimitSwitchRackCloseIsPressed(void)
{
  return HAL_GPIO_ReadPin(LimitSW_Rack_Close_GPIO_Port, LimitSW_Rack_Close_Pin)
      == GPIO_PIN_RESET;
}

static bool Board_UserSwitchIsPressed(void)
{
  return s_user_switch_interrupt_seen;
}

static void Board_ClearUserSwitchInterruptStatus(void)
{
  s_user_switch_interrupt_seen = false;
  s_user_switch_interrupt_count = 0u;
}

static void Board_UpdateSwingLimitLed(void)
{
  HAL_GPIO_WritePin(
      LED2_GPIO_Port,
      LED2_Pin,
      Board_LimitSwitchSwingIsPressed() ? GPIO_PIN_SET : GPIO_PIN_RESET
  );
}

static void Board_DCMotorSwingWaitForLimitRelease(void)
{
  while (Board_LimitSwitchSwingIsPressed()) {
    HAL_Delay(1);
  }
}

static void Board_DCMotorRackWaitForOpenLimitRelease(void)
{
  while (Board_LimitSwitchRackOpenIsPressed()) {
    HAL_Delay(1);
  }
}

static void Board_DCMotorRackWaitForCloseLimitRelease(void)
{
  while (Board_LimitSwitchRackCloseIsPressed()) {
    HAL_Delay(1);
  }
}

static void Board_StepperSetCurrent_mA(uint32_t current_mA)
{
  uint32_t code = TB67_DacCodeFromCurrent_mA(current_mA, BOARD_VDDA_MV);

  if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, code) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, code) != HAL_OK) {
    Error_Handler();
  }
}

static void Board_StepperEnable(bool enable)
{
  HAL_GPIO_WritePin(
      ENABLE_GPIO_Port,
      ENABLE_Pin,
      enable ? GPIO_PIN_SET : GPIO_PIN_RESET
  );
}

static void Board_StepperResetPulse(void)
{
  /*
   * RESET High -> internal electrical angle reset.
   * RESET Low  -> normal operation.
   * Keep ENABLE Low while doing this.
   */
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
}

static void Board_StepperScheduleNext(BoardStepperAxis *axis)
{
  if ((axis == NULL) || (axis->running == 0u) || (axis->half_ticks == 0u)) {
    return;
  }

  uint32_t ccr = __HAL_TIM_GET_COMPARE(axis->htim, axis->channel);
  ccr = (ccr + axis->half_ticks) & 0xFFFFu;

  __HAL_TIM_SET_COMPARE(axis->htim, axis->channel, ccr);
}

static uint32_t Board_StepperChannelToFlag(uint32_t channel)
{
  switch (channel) {
    case TIM_CHANNEL_1:
      return TIM_FLAG_CC1;
    case TIM_CHANNEL_2:
      return TIM_FLAG_CC2;
    case TIM_CHANNEL_3:
      return TIM_FLAG_CC3;
    case TIM_CHANNEL_4:
      return TIM_FLAG_CC4;
    default:
      return 0u;
  }
}

static bool Board_StepperMotorIsValid(BoardStepperMotor motor)
{
  return ((uint32_t)motor < BOARD_STEPPER_AXIS_COUNT);
}
