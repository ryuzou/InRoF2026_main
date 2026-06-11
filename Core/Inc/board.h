#ifndef BOARD_H
#define BOARD_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#ifndef BOARD_STEPPER_LEFT_FORWARD_PIN_STATE
#define BOARD_STEPPER_LEFT_FORWARD_PIN_STATE   GPIO_PIN_RESET
#endif

#ifndef BOARD_STEPPER_RIGHT_FORWARD_PIN_STATE
#define BOARD_STEPPER_RIGHT_FORWARD_PIN_STATE  GPIO_PIN_SET
#endif

typedef enum {
  BOARD_STEPPER_LEFT = 0,
  BOARD_STEPPER_RIGHT = 1
} BoardStepperMotor;

typedef enum {
  BOARD_STEPPER_DIR_FORWARD = 0,
  BOARD_STEPPER_DIR_REVERSE = 1
} BoardStepperDirection;

typedef enum {
  BOARD_DCMOTOR_SWING_DIR_0 = 0,
  BOARD_DCMOTOR_SWING_DIR_1 = 1
} BoardDCMotorSwingDirection;

typedef enum {
  BOARD_DCMOTOR_RACK_DIR_0 = 0,
  BOARD_DCMOTOR_RACK_DIR_1 = 1
} BoardDCMotorRackDirection;

#ifndef BOARD_DCMOTOR_SWING_RAISE_DIRECTION
#define BOARD_DCMOTOR_SWING_RAISE_DIRECTION  BOARD_DCMOTOR_SWING_DIR_1
#endif

#ifndef BOARD_DCMOTOR_SWING_LOWER_DIRECTION
#define BOARD_DCMOTOR_SWING_LOWER_DIRECTION  BOARD_DCMOTOR_SWING_DIR_0
#endif

#ifndef BOARD_DCMOTOR_RACK_OPEN_DIRECTION
#define BOARD_DCMOTOR_RACK_OPEN_DIRECTION  BOARD_DCMOTOR_RACK_DIR_0
#endif

#ifndef BOARD_DCMOTOR_RACK_CLOSE_DIRECTION
#define BOARD_DCMOTOR_RACK_CLOSE_DIRECTION  BOARD_DCMOTOR_RACK_DIR_1
#endif

void Board_Init(void);
uint32_t board_millis(void);
void board_idle(void);

bool board_canrpc_start(uint8_t own_addr);
bool board_canrpc_tx(uint16_t id, const uint8_t *data, uint8_t len);

typedef struct {
  uint32_t fifo1_count;
  uint32_t sensor_pub_count;
  uint32_t color_raw_count;
  uint32_t pose2d_count;
  uint32_t pub_short_count;
  uint32_t last_rx_t_ms;
  uint16_t last_id;
  uint8_t last_len;
  uint8_t last_topic;
  uint8_t last_pub_len;
} Board_CanIrqDebug;

void Board_CanIrqDebugSnapshot(Board_CanIrqDebug *snapshot);

uint32_t Board_MaxWheelSpeed_mm_s(void);

void Board_StepperStart(
    BoardStepperMotor motor,
    BoardStepperDirection direction,
    uint32_t step_hz
);
void Board_StepperStop(BoardStepperMotor motor);
void Board_StepperStopAll(void);

void Board_DCMotorSwingStart(BoardDCMotorSwingDirection direction);
void Board_DCMotorSwingStop(void);
void Board_DCMotorSwingMoveUntilLimit(BoardDCMotorSwingDirection direction);
void Board_DCMotorSwingRaiseUntilLimit(void);
void Board_DCMotorSwingLowerUntilLimit(void);

void Board_DCMotorRackStart(BoardDCMotorRackDirection direction);
void Board_DCMotorRackStop(void);
void Board_DCMotorRackOpenUntilLimit(void);
void Board_DCMotorRackCloseUntilLimit(void);

bool Board_FaultInterruptSeen(void);
uint32_t Board_FaultInterruptCount(void);
void Board_ClearFaultInterruptStatus(void);

bool Board_LimitSwitchSwingInterruptSeen(void);
uint32_t Board_LimitSwitchSwingInterruptCount(void);
void Board_ClearLimitSwitchSwingInterruptStatus(void);
void Board_WaitForLimitSwitchSwingInterrupt(void);

bool Board_LimitSwitchRackOpenInterruptSeen(void);
uint32_t Board_LimitSwitchRackOpenInterruptCount(void);
void Board_ClearLimitSwitchRackOpenInterruptStatus(void);
void Board_WaitForLimitSwitchRackOpenInterrupt(void);

bool Board_LimitSwitchRackCloseInterruptSeen(void);
uint32_t Board_LimitSwitchRackCloseInterruptCount(void);
void Board_ClearLimitSwitchRackCloseInterruptStatus(void);
void Board_WaitForLimitSwitchRackCloseInterrupt(void);

bool Board_StartSwitchInterruptSeen(void);
uint32_t Board_StartSwitchInterruptCount(void);
void Board_ClearStartSwitchInterruptStatus(void);
void Board_WaitForStartSwitchInterrupt(void);

void Board_LimitSwitchSwingInterruptHook(void);
void Board_LimitSwitchRackOpenInterruptHook(void);
void Board_LimitSwitchRackCloseInterruptHook(void);
void Board_StartSwitchInterruptHook(void);
void Board_FaultInterruptHook(void);

#endif /* BOARD_H */
