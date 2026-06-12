#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <stdint.h>

#ifndef ALGORITHM_ROBOT_WHEEL_DIAMETER_MM
#define ALGORITHM_ROBOT_WHEEL_DIAMETER_MM  58.0f
#endif

#ifndef ALGORITHM_ROBOT_TRACK_WIDTH_MM
#define ALGORITHM_ROBOT_TRACK_WIDTH_MM     212.0f
#endif

#ifndef ALGORITHM_STEPPER_FULL_STEPS_PER_REV
#define ALGORITHM_STEPPER_FULL_STEPS_PER_REV  200u
#endif

#ifndef ALGORITHM_STEPPER_MICROSTEPS
#define ALGORITHM_STEPPER_MICROSTEPS  32u
#endif

#ifndef ALGORITHM_ROBOT_WHEEL_STEPS_PER_REV
#define ALGORITHM_ROBOT_WHEEL_STEPS_PER_REV  \
  (ALGORITHM_STEPPER_FULL_STEPS_PER_REV * ALGORITHM_STEPPER_MICROSTEPS)
#endif

#ifndef ALGORITHM_COLOR_RAW_MAX
#define ALGORITHM_COLOR_RAW_MAX  4095u
#endif

#ifndef ALGORITHM_COLOR_NONE_VALUE_MAX
#define ALGORITHM_COLOR_NONE_VALUE_MAX  0.171f
#endif

#ifndef ALGORITHM_COLOR_NONE_HUE_MAX_DEG
#define ALGORITHM_COLOR_NONE_HUE_MAX_DEG  100.0f
#endif

#ifndef ALGORITHM_COLOR_RED_HUE_DEG
#define ALGORITHM_COLOR_RED_HUE_DEG  360.0f
#endif

#ifndef ALGORITHM_COLOR_YELLOW_HUE_DEG
#define ALGORITHM_COLOR_YELLOW_HUE_DEG  30.0f
#endif

#ifndef ALGORITHM_COLOR_BLUE_HUE_DEG
#define ALGORITHM_COLOR_BLUE_HUE_DEG  240.0f
#endif

typedef enum {
  ALGORITHM_BALL_COLOR_NONE = 0,
  ALGORITHM_BALL_COLOR_RED = 1,
  ALGORITHM_BALL_COLOR_YELLOW = 2,
  ALGORITHM_BALL_COLOR_BLUE = 3,
} Algorithm_BallColor;

typedef struct {
  float hue_deg;
  Algorithm_BallColor color;
} Algorithm_ColorDetection;

uint32_t Algorithm_RobotWheelSpeedToStepHz_mm_s(uint32_t speed_mm_s);
void Algorithm_RobotSetWheelSpeed_mm_s(int32_t left_mm_s, int32_t right_mm_s);
void Algorithm_RobotSetVelocity(float linear_mm_s, float angular_rad_s);
void Algorithm_RobotDriveStraight_mm_s(int32_t speed_mm_s);
void Algorithm_RobotTurnInPlace_rad_s(float angular_rad_s);
void Algorithm_RobotStop(void);
Algorithm_ColorDetection Algorithm_DetectBallColorFromRgb(
    uint16_t r,
    uint16_t g,
    uint16_t b
);
const char *Algorithm_BallColorName(Algorithm_BallColor color);

#endif /* ALGORITHM_H */
