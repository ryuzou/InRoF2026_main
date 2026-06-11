#include "algorithm.h"

#include "board.h"

#define ALGORITHM_ROBOT_PI  3.14159265358979323846f

static uint32_t Algorithm_RobotAbsInt32(int32_t value);
static uint32_t Algorithm_RobotClampWheelSpeed_mm_s(uint32_t speed_mm_s);
static void Algorithm_RobotSetOneWheelSpeed_mm_s(
    BoardStepperMotor motor,
    int32_t speed_mm_s
);

uint32_t Algorithm_RobotWheelSpeedToStepHz_mm_s(uint32_t speed_mm_s)
{
  if (speed_mm_s == 0u) {
    return 0u;
  }

  float wheel_circumference_mm = ALGORITHM_ROBOT_WHEEL_DIAMETER_MM
      * ALGORITHM_ROBOT_PI;
  float step_hz = ((float)speed_mm_s
      * (float)ALGORITHM_ROBOT_WHEEL_STEPS_PER_REV) / wheel_circumference_mm;

  if (step_hz < 1.0f) {
    return 1u;
  }

  return (uint32_t)(step_hz + 0.5f);
}

void Algorithm_RobotSetWheelSpeed_mm_s(int32_t left_mm_s, int32_t right_mm_s)
{
  Algorithm_RobotSetOneWheelSpeed_mm_s(BOARD_STEPPER_LEFT, left_mm_s);
  Algorithm_RobotSetOneWheelSpeed_mm_s(BOARD_STEPPER_RIGHT, right_mm_s);
}

void Algorithm_RobotSetVelocity(float linear_mm_s, float angular_rad_s)
{
  float half_track_mm = ALGORITHM_ROBOT_TRACK_WIDTH_MM * 0.5f;
  float left_mm_s = linear_mm_s - angular_rad_s * half_track_mm;
  float right_mm_s = linear_mm_s + angular_rad_s * half_track_mm;

  Algorithm_RobotSetWheelSpeed_mm_s((int32_t)left_mm_s, (int32_t)right_mm_s);
}

void Algorithm_RobotDriveStraight_mm_s(int32_t speed_mm_s)
{
  Algorithm_RobotSetWheelSpeed_mm_s(speed_mm_s, speed_mm_s);
}

void Algorithm_RobotTurnInPlace_rad_s(float angular_rad_s)
{
  Algorithm_RobotSetVelocity(0.0f, angular_rad_s);
}

void Algorithm_RobotStop(void)
{
  Board_StepperStopAll();
}

static uint32_t Algorithm_RobotAbsInt32(int32_t value)
{
  if (value >= 0) {
    return (uint32_t)value;
  }

  return (uint32_t)(-(value + 1)) + 1u;
}

static uint32_t Algorithm_RobotClampWheelSpeed_mm_s(uint32_t speed_mm_s)
{
  uint32_t max_speed_mm_s = Board_MaxWheelSpeed_mm_s();

  if (speed_mm_s > max_speed_mm_s) {
    return max_speed_mm_s;
  }

  return speed_mm_s;
}

static void Algorithm_RobotSetOneWheelSpeed_mm_s(
    BoardStepperMotor motor,
    int32_t speed_mm_s
)
{
  uint32_t step_hz = Algorithm_RobotWheelSpeedToStepHz_mm_s(
      Algorithm_RobotClampWheelSpeed_mm_s(Algorithm_RobotAbsInt32(speed_mm_s))
  );

  if (step_hz == 0u) {
    Board_StepperStop(motor);
    return;
  }

  Board_StepperStart(
      motor,
      (speed_mm_s >= 0) ? BOARD_STEPPER_DIR_FORWARD : BOARD_STEPPER_DIR_REVERSE,
      step_hz
  );
}
