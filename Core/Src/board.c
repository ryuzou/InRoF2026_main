#include "board.h"

#define BOARD_STEPPER_TIM_HZ       1000000u  /* TIM3: 80 MHz / (79 + 1) */
#define BOARD_VDDA_MV              3300u
#define BOARD_STEPPER_CURRENT_MA   1000u
#define BOARD_STEPPER_AXIS_COUNT   2u
#define BOARD_DCMOTOR_DUTY_SCALE   1000u

#ifndef BOARD_DCMOTOR_SWING_DIR0_DUTY_PERMILLE
#define BOARD_DCMOTOR_SWING_DIR0_DUTY_PERMILLE  500u
#endif

#ifndef BOARD_DCMOTOR_SWING_DIR1_DUTY_PERMILLE
#define BOARD_DCMOTOR_SWING_DIR1_DUTY_PERMILLE  500u
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
static volatile uint8_t s_dcmotor_swing_running = 0u;
static volatile bool s_start_switch_interrupt_seen = false;
static volatile uint32_t s_start_switch_interrupt_count = 0u;
static volatile bool s_user_switch_interrupt_seen = false;
static volatile uint32_t s_user_switch_interrupt_count = 0u;

static uint32_t TB67_DacCodeFromCurrent_mA(uint32_t current_mA, uint32_t vdda_mV);
static uint32_t Board_DCMotorDutyToPulse(TIM_HandleTypeDef *htim, uint32_t duty_permille);
static bool Board_DCMotorSwingDirectionIsValid(BoardDCMotorSwingDirection direction);
static uint32_t Board_DCMotorSwingDutyPermille(BoardDCMotorSwingDirection direction);
static bool Board_LimitSwitchSwingIsPressed(void);
static bool Board_UserSwitchIsPressed(void);
static void Board_ClearUserSwitchInterruptStatus(void);
static void Board_UpdateSwingLimitLed(void);
static void Board_DCMotorSwingWaitForLimitRelease(void);
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

bool Board_StartSwitchInterruptSeen(void)
{
  return s_start_switch_interrupt_seen;
}

uint32_t Board_StartSwitchInterruptCount(void)
{
  return s_start_switch_interrupt_count;
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

static bool Board_LimitSwitchSwingIsPressed(void)
{
  return HAL_GPIO_ReadPin(LimitSW_Swing_GPIO_Port, LimitSW_Swing_Pin) == GPIO_PIN_RESET;
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
