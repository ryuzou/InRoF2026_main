#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <stdbool.h>
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

#define ALGORITHM_TSD10_CHANNELS 3u

#ifndef ALGORITHM_COLOR_MEASURE_ATIME
#define ALGORITHM_COLOR_MEASURE_ATIME  0x80
#endif

#ifndef ALGORITHM_COLOR_MEASURE_GAIN
#define ALGORITHM_COLOR_MEASURE_GAIN   1u
#endif

#ifndef ALGORITHM_COLOR_MEASURE_FLAGS
#define ALGORITHM_COLOR_MEASURE_FLAGS  0u
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

typedef struct {
  bool valid[ALGORITHM_TSD10_CHANNELS];
  bool out_of_range[ALGORITHM_TSD10_CHANNELS];
  uint8_t rpc_result[ALGORITHM_TSD10_CHANNELS];
  uint16_t distance_mm[ALGORITHM_TSD10_CHANNELS];
  uint8_t seq;
  uint8_t flags;
  uint32_t sensor_t_ms;
  uint32_t rx_t_ms;
} Algorithm_Tsd10;

typedef struct {
  bool valid;
  uint8_t seq;
  uint8_t rpc_result;
  uint32_t sensor_t_ms;
  uint32_t rx_t_ms;
  uint16_t clear;
  uint16_t red;
  uint16_t green;
  uint16_t blue;
  uint8_t atime;
  uint8_t gain;
  uint8_t led_on_ms;
  uint8_t flags;
} Algorithm_ColorRaw;

typedef struct {
  Algorithm_Tsd10 tsd;
  Algorithm_ColorRaw color;
  int tsd_status;
  int color_status;
} Algorithm_SensorSample;

void Algorithm_SensorInit(void);
void Algorithm_SensorOnCanrpcPublish(uint8_t node, uint8_t topic, const uint8_t *data, uint8_t len);
uint32_t Algorithm_RobotWheelSpeedToStepHz_mm_s(uint32_t speed_mm_s);
void Algorithm_RobotSetWheelSpeed_mm_s(int32_t left_mm_s, int32_t right_mm_s);
void Algorithm_RobotSetVelocity(float linear_mm_s, float angular_rad_s);
void Algorithm_RobotDriveStraight_mm_s(int32_t speed_mm_s);
void Algorithm_RobotTurnInPlace_rad_s(float angular_rad_s);
void Algorithm_RobotStop(void);
int Algorithm_ServoOpenCloseBlocking(uint8_t *rpc_result, uint32_t timeout_ms);
int Algorithm_ReadTsd10Blocking(Algorithm_Tsd10 *out, uint32_t timeout_ms);
int Algorithm_ReadColorRawBlocking(Algorithm_ColorRaw *out, uint32_t timeout_ms);
int Algorithm_ReadBallColorBlocking(
    Algorithm_ColorDetection *out,
    Algorithm_ColorRaw *raw_out,
    uint32_t timeout_ms
);
int Algorithm_ReadSensorSampleBlocking(Algorithm_SensorSample *out, uint32_t timeout_ms);
Algorithm_ColorDetection Algorithm_DetectBallColorFromRgb(
    uint16_t r,
    uint16_t g,
    uint16_t b
);
const char *Algorithm_BallColorName(Algorithm_BallColor color);

#endif /* ALGORITHM_H */
