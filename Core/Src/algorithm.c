#include "algorithm.h"

#include "board.h"
#include "canrpc.h"
#include "main.h"
#include "robot_can.h"
#include <string.h>

#define ALGORITHM_ROBOT_PI  3.14159265358979323846f

#define CMD_TSD_READ             0x10u
#define CMD_COLOR_MEASURE        0x20u

#define TOPIC_COLOR_RAW          0x30u

#define SENSOR_COLOR_ARG(atime, gain, flags) \
  ((int32_t)((uint32_t)(atime) | ((uint32_t)(gain) << 8) | ((uint32_t)(flags) << 16)))

static volatile Algorithm_ColorRaw s_color_raw;

static uint32_t Algorithm_RobotAbsInt32(int32_t value);
static uint32_t Algorithm_RobotClampWheelSpeed_mm_s(uint32_t speed_mm_s);
static void Algorithm_RobotSetOneWheelSpeed_mm_s(
    BoardStepperMotor motor,
    int32_t speed_mm_s
);
static bool Algorithm_TimeReached(uint32_t now_ms, uint32_t deadline_ms);
static bool Algorithm_TimeNotBefore(uint32_t t_ms, uint32_t start_ms);
static uint16_t Algorithm_GetU16Le(const uint8_t *p);
static uint32_t Algorithm_GetU32Le(const uint8_t *p);
static void Algorithm_ColorRawSnapshot(Algorithm_ColorRaw *out);
static uint16_t Algorithm_ColorClampRaw(uint16_t value);
static float Algorithm_ColorMinFloat(float a, float b);
static float Algorithm_ColorMaxFloat(float a, float b);
static float Algorithm_ColorAbsFloat(float value);
static float Algorithm_ColorHueFromRgb01(float red, float green, float blue);
static Algorithm_BallColor Algorithm_ColorClassify(float hue_deg, float value);
static Algorithm_BallColor Algorithm_ColorNearestHue(float hue_deg);
static float Algorithm_ColorRedHueDistance(float hue_deg);

void Algorithm_SensorInit(void)
{
  memset((void *)&s_color_raw, 0, sizeof(s_color_raw));
  s_color_raw.rpc_result = CANRPC_RES_INVALID;
}

void Algorithm_SensorOnCanrpcPublish(uint8_t node, uint8_t topic, const uint8_t *data, uint8_t len)
{
  if (node != NODE_SENSOR || data == NULL) {
    return;
  }

  if (topic == TOPIC_COLOR_RAW && len >= 17u) {
    s_color_raw.seq = data[0];
    s_color_raw.rpc_result = CANRPC_OK;
    s_color_raw.sensor_t_ms = Algorithm_GetU32Le(&data[1]);
    s_color_raw.rx_t_ms = HAL_GetTick();
    s_color_raw.clear = Algorithm_GetU16Le(&data[5]);
    s_color_raw.red = Algorithm_GetU16Le(&data[7]);
    s_color_raw.green = Algorithm_GetU16Le(&data[9]);
    s_color_raw.blue = Algorithm_GetU16Le(&data[11]);
    s_color_raw.atime = data[13];
    s_color_raw.gain = data[14];
    s_color_raw.led_on_ms = data[15];
    s_color_raw.flags = data[16];
    s_color_raw.valid = true;
  }
}

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

int Algorithm_ServoOpenCloseBlocking(uint8_t *rpc_result, uint32_t timeout_ms)
{
  if (rpc_result != NULL) {
    *rpc_result = CANRPC_RES_INVALID;
  }

  int handle = canrpc_call(NODE_SERVO, CMD_SERVO_OPEN_CLOSE, 0);
  if (handle < 0) {
    if (rpc_result != NULL) {
      *rpc_result = CANRPC_RES_BUSY;
    }
    return CANRPC_ERR_PARAM;
  }

  int rc = canrpc_wait(CANRPC_H(handle), timeout_ms);
  if (rpc_result != NULL) {
    *rpc_result = canrpc_result(handle);
  }
  return rc;
}

int Algorithm_ReadTsd10Blocking(Algorithm_Tsd10 *out, uint32_t timeout_ms)
{
  if (out == NULL) {
    return CANRPC_ERR_PARAM;
  }

  Algorithm_Tsd10 tsd;
  memset(&tsd, 0, sizeof(tsd));
  for (uint8_t ch = 0; ch < ALGORITHM_TSD10_CHANNELS; ch++) {
    tsd.rpc_result[ch] = CANRPC_RES_INVALID;
  }

  int handles[ALGORITHM_TSD10_CHANNELS] = {-1, -1, -1};
  uint32_t wait_mask = 0u;

  for (uint8_t ch = 0; ch < ALGORITHM_TSD10_CHANNELS; ch++) {
    handles[ch] = canrpc_call(NODE_SENSOR, CMD_TSD_READ, (int32_t)ch);
    if (handles[ch] >= 0) {
      wait_mask |= CANRPC_H(handles[ch]);
    } else {
      tsd.rpc_result[ch] = CANRPC_RES_BUSY;
    }
  }

  int rc = (wait_mask != 0u) ? canrpc_wait(wait_mask, timeout_ms) : CANRPC_ERR_PARAM;
  tsd.rx_t_ms = HAL_GetTick();

  for (uint8_t ch = 0; ch < ALGORITHM_TSD10_CHANNELS; ch++) {
    if (handles[ch] < 0) {
      continue;
    }

    tsd.rpc_result[ch] = canrpc_result(handles[ch]);
    tsd.valid[ch] = (tsd.rpc_result[ch] == CANRPC_OK);
    if (tsd.valid[ch]) {
      tsd.distance_mm[ch] = (uint16_t)canrpc_ret(handles[ch]);
    }
  }

  *out = tsd;
  return rc;
}

int Algorithm_ReadColorRawBlocking(Algorithm_ColorRaw *out, uint32_t timeout_ms)
{
  if (out == NULL) {
    return CANRPC_ERR_PARAM;
  }

  Algorithm_ColorRaw color;
  memset(&color, 0, sizeof(color));
  color.rpc_result = CANRPC_RES_INVALID;

  uint32_t start_ms = board_millis();
  uint32_t deadline_ms = start_ms + timeout_ms;
  int handle = canrpc_call(
      NODE_SENSOR,
      CMD_COLOR_MEASURE,
      SENSOR_COLOR_ARG(
          ALGORITHM_COLOR_MEASURE_ATIME,
          ALGORITHM_COLOR_MEASURE_GAIN,
          ALGORITHM_COLOR_MEASURE_FLAGS
      )
  );
  if (handle < 0) {
    color.rpc_result = CANRPC_RES_BUSY;
    *out = color;
    return CANRPC_ERR_PARAM;
  }

  int rc = canrpc_wait(CANRPC_H(handle), timeout_ms);
  color.rpc_result = canrpc_result(handle);
  if (rc != 0 || color.rpc_result != CANRPC_OK) {
    *out = color;
    return (rc != 0) ? rc : (int)color.rpc_result;
  }

  uint8_t expected_seq = (uint8_t)canrpc_ret(handle);
  for (;;) {
    Algorithm_ColorRawSnapshot(&color);
    if (color.valid && color.seq == expected_seq && Algorithm_TimeNotBefore(color.rx_t_ms, start_ms)) {
      *out = color;
      return 0;
    }

    if (Algorithm_TimeReached(board_millis(), deadline_ms)) {
      color.valid = false;
      color.seq = expected_seq;
      color.rpc_result = CANRPC_OK;
      *out = color;
      return CANRPC_ERR_TIMEOUT;
    }

    board_idle();
  }
}

int Algorithm_ReadSensorSampleBlocking(Algorithm_SensorSample *out, uint32_t timeout_ms)
{
  if (out == NULL) {
    return CANRPC_ERR_PARAM;
  }

  Algorithm_SensorSample sample;
  memset(&sample, 0, sizeof(sample));

  sample.tsd_status = Algorithm_ReadTsd10Blocking(&sample.tsd, timeout_ms);
  sample.color_status = Algorithm_ReadColorRawBlocking(&sample.color, timeout_ms);

  *out = sample;
  return (sample.tsd_status != 0) ? sample.tsd_status : sample.color_status;
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

static bool Algorithm_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
  return (int32_t)(now_ms - deadline_ms) >= 0;
}

static bool Algorithm_TimeNotBefore(uint32_t t_ms, uint32_t start_ms)
{
  return (int32_t)(t_ms - start_ms) >= 0;
}

static uint16_t Algorithm_GetU16Le(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t Algorithm_GetU32Le(const uint8_t *p)
{
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static void Algorithm_ColorRawSnapshot(Algorithm_ColorRaw *out)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  out->valid = s_color_raw.valid;
  out->seq = s_color_raw.seq;
  out->rpc_result = s_color_raw.rpc_result;
  out->sensor_t_ms = s_color_raw.sensor_t_ms;
  out->rx_t_ms = s_color_raw.rx_t_ms;
  out->clear = s_color_raw.clear;
  out->red = s_color_raw.red;
  out->green = s_color_raw.green;
  out->blue = s_color_raw.blue;
  out->atime = s_color_raw.atime;
  out->gain = s_color_raw.gain;
  out->led_on_ms = s_color_raw.led_on_ms;
  out->flags = s_color_raw.flags;
  if (primask == 0u) {
    __enable_irq();
  }
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
