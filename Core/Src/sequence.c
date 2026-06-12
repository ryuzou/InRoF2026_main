#include "sequence.h"

#include "algorithm.h"
#include "main.h"
#include "robot_control.h"

#include <stdbool.h>
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

#define SEQUENCE_BALL_READ_X_MM  300.0f
#define SEQUENCE_BALL_READ_Y_MM  250.0f
#define SEQUENCE_DROP_X_MM       300.0f
#define SEQUENCE_DROP_H_DEG      90.0f

#define SEQUENCE_RED_DROP_Y_MM     250.0f
#define SEQUENCE_YELLOW_DROP_Y_MM  900.0f
#define SEQUENCE_BLUE_DROP_Y_MM    1550.0f

static void Sequence_WaitForRobotCommand(void);
static bool Sequence_BallDropYMm(Algorithm_BallColor color, float *drop_y_mm);

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

static void Sequence_WaitForRobotCommand(void)
{
  while (!RobotControl_IsCommandComplete()) {
  }
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
