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

void Board_Init(void);

void Board_StepperStart(
    BoardStepperMotor motor,
    BoardStepperDirection direction,
    uint32_t step_hz
);
void Board_StepperStop(BoardStepperMotor motor);
void Board_StepperStopAll(void);

bool Board_FaultInterruptSeen(void);
uint32_t Board_FaultInterruptCount(void);
void Board_ClearFaultInterruptStatus(void);

void Board_FaultInterruptHook(void);

#endif /* BOARD_H */
