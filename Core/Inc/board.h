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

#ifndef BOARD_DCMOTOR_SWING_RAISE_DIRECTION
#define BOARD_DCMOTOR_SWING_RAISE_DIRECTION  BOARD_DCMOTOR_SWING_DIR_1
#endif

#ifndef BOARD_DCMOTOR_SWING_LOWER_DIRECTION
#define BOARD_DCMOTOR_SWING_LOWER_DIRECTION  BOARD_DCMOTOR_SWING_DIR_0
#endif

void Board_Init(void);

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

bool Board_FaultInterruptSeen(void);
uint32_t Board_FaultInterruptCount(void);
void Board_ClearFaultInterruptStatus(void);

bool Board_LimitSwitchSwingInterruptSeen(void);
uint32_t Board_LimitSwitchSwingInterruptCount(void);
void Board_ClearLimitSwitchSwingInterruptStatus(void);
void Board_WaitForLimitSwitchSwingInterrupt(void);

bool Board_StartSwitchInterruptSeen(void);
uint32_t Board_StartSwitchInterruptCount(void);
void Board_ClearStartSwitchInterruptStatus(void);
void Board_WaitForStartSwitchInterrupt(void);

void Board_LimitSwitchSwingInterruptHook(void);
void Board_StartSwitchInterruptHook(void);
void Board_FaultInterruptHook(void);

#endif /* BOARD_H */
