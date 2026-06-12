#include "algorithm.h"

#include "board.h"

#define ALGORITHM_ROBOT_PI  3.14159265358979323846f

static uint32_t Algorithm_RobotAbsInt32(int32_t value);
static uint32_t Algorithm_RobotClampWheelSpeed_mm_s(uint32_t speed_mm_s);
static void Algorithm_RobotSetOneWheelSpeed_mm_s(
    BoardStepperMotor motor,
    int32_t speed_mm_s
);
static uint16_t Algorithm_ColorClampRaw(uint16_t value);
static float Algorithm_ColorMinFloat(float a, float b);
static float Algorithm_ColorMaxFloat(float a, float b);
static float Algorithm_ColorAbsFloat(float value);
static float Algorithm_ColorHueFromRgb01(float red, float green, float blue);
static Algorithm_BallColor Algorithm_ColorClassify(float hue_deg, float value);
static Algorithm_BallColor Algorithm_ColorNearestHue(float hue_deg);
static float Algorithm_ColorRedHueDistance(float hue_deg);

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

Algorithm_ColorDetection Algorithm_DetectBallColorFromRgb(
    uint16_t r,
    uint16_t g,
    uint16_t b
)
{
  r = Algorithm_ColorClampRaw(r);
  g = Algorithm_ColorClampRaw(g);
  b = Algorithm_ColorClampRaw(b);

  float red = (float)r / (float)ALGORITHM_COLOR_RAW_MAX;
  float green = (float)g / (float)ALGORITHM_COLOR_RAW_MAX;
  float blue = (float)b / (float)ALGORITHM_COLOR_RAW_MAX;
  float max_val = Algorithm_ColorMaxFloat(red, Algorithm_ColorMaxFloat(green, blue));
  float hue_deg = Algorithm_ColorHueFromRgb01(red, green, blue);

  Algorithm_ColorDetection detection = {
      .hue_deg = hue_deg,
      .color = Algorithm_ColorClassify(hue_deg, max_val),
  };
  return detection;
}

const char *Algorithm_BallColorName(Algorithm_BallColor color)
{
  switch (color) {
    case ALGORITHM_BALL_COLOR_NONE:
      return "none";
    case ALGORITHM_BALL_COLOR_RED:
      return "red";
    case ALGORITHM_BALL_COLOR_YELLOW:
      return "yellow";
    case ALGORITHM_BALL_COLOR_BLUE:
      return "blue";
    default:
      return "unknown";
  }
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

static uint16_t Algorithm_ColorClampRaw(uint16_t value)
{
  if (value > ALGORITHM_COLOR_RAW_MAX) {
    return (uint16_t)ALGORITHM_COLOR_RAW_MAX;
  }

  return value;
}

static float Algorithm_ColorMinFloat(float a, float b)
{
  return (a < b) ? a : b;
}

static float Algorithm_ColorMaxFloat(float a, float b)
{
  return (a > b) ? a : b;
}

static float Algorithm_ColorAbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float Algorithm_ColorHueFromRgb01(float red, float green, float blue)
{
  float min_val = Algorithm_ColorMinFloat(red, Algorithm_ColorMinFloat(green, blue));
  float max_val = Algorithm_ColorMaxFloat(red, Algorithm_ColorMaxFloat(green, blue));
  float chroma = max_val - min_val;
  float hue_deg = 0.0f;

  if (chroma <= 0.0f) {
    return 0.0f;
  }

  if ((blue < red) && (blue < green)) {
    hue_deg = (60.0f * (green - red) / chroma) + 60.0f;
  } else if ((red < blue) && (red < green)) {
    hue_deg = (60.0f * (blue - green) / chroma) + 180.0f;
  } else if ((green < red) && (green < blue)) {
    hue_deg = (60.0f * (red - blue) / chroma) + 300.0f;
  }

  if (hue_deg < 0.0f) {
    hue_deg += 360.0f;
  }

  return hue_deg;
}

static Algorithm_BallColor Algorithm_ColorClassify(float hue_deg, float value)
{
  if ((value < ALGORITHM_COLOR_NONE_VALUE_MAX)
      && (hue_deg < ALGORITHM_COLOR_NONE_HUE_MAX_DEG)) {
    return ALGORITHM_BALL_COLOR_NONE;
  }

  return Algorithm_ColorNearestHue(hue_deg);
}

static Algorithm_BallColor Algorithm_ColorNearestHue(float hue_deg)
{
  float diff_red = Algorithm_ColorRedHueDistance(hue_deg);
  float diff_yellow = Algorithm_ColorAbsFloat(hue_deg - ALGORITHM_COLOR_YELLOW_HUE_DEG);
  float diff_blue = Algorithm_ColorAbsFloat(hue_deg - ALGORITHM_COLOR_BLUE_HUE_DEG);

  if ((diff_red < diff_yellow) && (diff_red < diff_blue)) {
    return ALGORITHM_BALL_COLOR_RED;
  }

  if ((diff_yellow < diff_red) && (diff_yellow < diff_blue)) {
    return ALGORITHM_BALL_COLOR_YELLOW;
  }

  return ALGORITHM_BALL_COLOR_BLUE;
}

static float Algorithm_ColorRedHueDistance(float hue_deg)
{
  return Algorithm_ColorMinFloat(
      Algorithm_ColorAbsFloat(hue_deg - ALGORITHM_COLOR_RED_HUE_DEG),
      Algorithm_ColorAbsFloat((hue_deg + 360.0f) - ALGORITHM_COLOR_RED_HUE_DEG)
  );
}
