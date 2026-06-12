#include "sequence.h"

#include "algorithm.h"
#include "main.h"
#include "robot_control.h"
#include "board.h"
#include "canrpc.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifndef SEQUENCE_MAX_STORED_BALLS
#define SEQUENCE_MAX_STORED_BALLS  2
#endif

#ifndef SEQUENCE_BALL_LOAD_RETRY_COUNT
#define SEQUENCE_BALL_LOAD_RETRY_COUNT  4
#endif

#ifndef SEQUENCE_BALL_LOAD_RETRY_DELAY_MS
#define SEQUENCE_BALL_LOAD_RETRY_DELAY_MS  400u
#endif

#ifndef SEQUENCE_COLOR_READ_TIMEOUT_MS
#define SEQUENCE_COLOR_READ_TIMEOUT_MS  1000u
#endif

#ifndef SEQUENCE_SERVO_TIMEOUT_MS
#define SEQUENCE_SERVO_TIMEOUT_MS  100u
#endif

#ifndef SEQUENCE_TSD10_READ_TIMEOUT_MS
#define SEQUENCE_TSD10_READ_TIMEOUT_MS  (CANRPC_DONE_HOLD_MS + 1000u)
#endif

#define SEQUENCE_BALL_READ_X_MM  300.0f
#define SEQUENCE_BALL_READ_Y_MM  250.0f
#define SEQUENCE_DROP_X_MM       300.0f
#define SEQUENCE_DROP_H_DEG      90.0f

#define SEQUENCE_TSD10_BACK_RIGHT_INDEX  1u
#define SEQUENCE_TSD10_BACK_LEFT_INDEX   2u
#define SEQUENCE_TSD10_BACK_SPAN_MM      144.0f

#define SEQUENCE_RED_DROP_Y_MM     250.0f
#define SEQUENCE_YELLOW_DROP_Y_MM  900.0f
#define SEQUENCE_BLUE_DROP_Y_MM    1550.0f

static bool Sequence_BallDropYMm(Algorithm_BallColor color, float *drop_y_mm);
static int32_t Sequence_RoundToI32(float value);

void Sequence_CollectBalls(void) {
  Board_DCMotorRackOpenUntilLimit();
  Board_DCMotorSwingLowerUntilLimit();
  Board_DCMotorRackCloseUntilLimit();
  Board_DCMotorSwingRaiseUntilLimit();
}

void Sequence_CalibrateHeadingWithTsd10YWall(void)
{
  Algorithm_Tsd10 tsd;
  (void)Algorithm_ReadTsd10Blocking(&tsd, SEQUENCE_TSD10_READ_TIMEOUT_MS);

  if (!tsd.valid[SEQUENCE_TSD10_BACK_RIGHT_INDEX]
      || !tsd.valid[SEQUENCE_TSD10_BACK_LEFT_INDEX]) {
    Error_Handler();
  }

  int32_t x1_mm = (int32_t)tsd.distance_mm[SEQUENCE_TSD10_BACK_RIGHT_INDEX]
      - (int32_t)tsd.distance_mm[SEQUENCE_TSD10_BACK_LEFT_INDEX];
  float heading_rad = -atan2f((float)x1_mm, SEQUENCE_TSD10_BACK_SPAN_MM);
  int32_t heading_mrad = Sequence_RoundToI32(heading_rad * 1000.0f);

  RobotControl_SetCurrentPose(NULL, NULL, heading_mrad);
}

void Sequence_CallibrateRP1(void)
{
  Sequence_CalibrateHeadingWithTsd10YWall();

  if (RobotControl_IssueTurnTo_deg(0.0f) != ROBOT_CONTROL_COMMAND_OK) {
    Error_Handler();
  }
  Sequence_WaitForRobotCommand();

  Algorithm_Tsd10 tsd;
  (void)Algorithm_ReadTsd10Blocking(&tsd, SEQUENCE_TSD10_READ_TIMEOUT_MS);
  if (!tsd.valid[0]
      || !tsd.valid[SEQUENCE_TSD10_BACK_RIGHT_INDEX]
      || !tsd.valid[SEQUENCE_TSD10_BACK_LEFT_INDEX]) {
    Error_Handler();
  }

  float x_2 = tsd.distance_mm[0];
  float x_3 = (tsd.distance_mm[1] + tsd.distance_mm[2]) / 2.0f;


  float correct_x = 500 - (86 + x_2);
  float correct_y = -500 + (84 + x_3);
  RobotControl_SetCurrentPose(correct_x, correct_y, 0);
  // Sequence_CalibrateHeadingWithTsd10YWall();
}

void Sequence_CallibrateRP2(void)
{
  (void)RobotControl_IssueMoveToPose_mm_deg(NULL, NULL, 0);
  Sequence_CalibrateHeadingWithTsd10YWall();

  if (RobotControl_IssueTurnTo_deg(0.0f) != ROBOT_CONTROL_COMMAND_OK) {
    Error_Handler();
  }
  Sequence_WaitForRobotCommand();

  Algorithm_Tsd10 tsd;
  (void)Algorithm_ReadTsd10Blocking(&tsd, SEQUENCE_TSD10_READ_TIMEOUT_MS);
  if (!tsd.valid[0]
      || !tsd.valid[SEQUENCE_TSD10_BACK_RIGHT_INDEX]
      || !tsd.valid[SEQUENCE_TSD10_BACK_LEFT_INDEX]) {
    Error_Handler();
      }

  float x_2 = tsd.distance_mm[0];
  float x_3 = (tsd.distance_mm[1] + tsd.distance_mm[2]) / 2.0f;


  float correct_x = 1340 - (86 + x_2);
  float correct_y = 0 + (84 + x_3);
  RobotControl_SetCurrentPose(correct_x, correct_y, 0);
  // Sequence_CalibrateHeadingWithTsd10YWall();
}

void Sequence_PlaceStoredBalls(void)
{
  (void)RobotControl_IssueMoveToPose_mm_deg(
      SEQUENCE_BALL_READ_X_MM,
      SEQUENCE_BALL_READ_Y_MM,
      NULL
  );
  Sequence_WaitForRobotCommand();

  int placed_balls = 0;
  int empty_read_count = 0;
  while (placed_balls < SEQUENCE_MAX_STORED_BALLS) {
    Algorithm_ColorDetection color_detection;
    Algorithm_ColorRaw color_raw;
    int color_status = Algorithm_ReadBallColorBlocking(
        &color_detection,
        &color_raw,
        SEQUENCE_COLOR_READ_TIMEOUT_MS
    );
    if (color_status != 0 || color_detection.color == ALGORITHM_BALL_COLOR_NONE) {
      empty_read_count++;
      if (empty_read_count >= SEQUENCE_BALL_LOAD_RETRY_COUNT) {
        break;
      }

      HAL_Delay(SEQUENCE_BALL_LOAD_RETRY_DELAY_MS);
      continue;
    }

    float drop_y_mm = 0.0f;
    if (!Sequence_BallDropYMm(color_detection.color, &drop_y_mm)) {
      continue;
    }

    empty_read_count = 0;
    (void)RobotControl_IssueMoveToPose_mm_deg(
        SEQUENCE_DROP_X_MM,
        drop_y_mm,
        SEQUENCE_DROP_H_DEG
    );
    Sequence_WaitForRobotCommand();
    (void)Algorithm_ServoOpenCloseBlocking(NULL, SEQUENCE_SERVO_TIMEOUT_MS);

    placed_balls++;
    HAL_Delay(SEQUENCE_BALL_LOAD_RETRY_DELAY_MS);
  }

  (void)RobotControl_IssueMoveToPose_mm_deg(
      SEQUENCE_BALL_READ_X_MM,
      SEQUENCE_BALL_READ_Y_MM,
      SEQUENCE_DROP_H_DEG
  );
  Sequence_WaitForRobotCommand();
}

void Sequence_WaitForRobotCommand(void)
{
  (void)RobotControl_WaitForCommandComplete(ROBOT_CONTROL_WAIT_FOREVER_MS);
}

void Sequence_IssueMoveToRP2(void) {
  (void)RobotControl_IssueMoveToPose_mm_deg(1340, 162.5, NULL);
}

static bool Sequence_BallDropYMm(Algorithm_BallColor color, float *drop_y_mm)
{
  if (drop_y_mm == NULL) {
    return false;
  }

  switch (color) {
    case ALGORITHM_BALL_COLOR_RED:
      *drop_y_mm = SEQUENCE_RED_DROP_Y_MM;
      return true;

    case ALGORITHM_BALL_COLOR_YELLOW:
      *drop_y_mm = SEQUENCE_YELLOW_DROP_Y_MM;
      return true;

    case ALGORITHM_BALL_COLOR_BLUE:
      *drop_y_mm = SEQUENCE_BLUE_DROP_Y_MM;
      return true;

    default:
      return false;
  }
}

static int32_t Sequence_RoundToI32(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}
