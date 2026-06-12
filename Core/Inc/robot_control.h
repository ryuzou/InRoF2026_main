#ifndef ROBOT_CONTROL_H_INCLUDED
#define ROBOT_CONTROL_H_INCLUDED

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOT_CONTROL_TSD10_CHANNELS 3u

typedef struct {
  bool valid[ROBOT_CONTROL_TSD10_CHANNELS];
  bool out_of_range[ROBOT_CONTROL_TSD10_CHANNELS];
  uint8_t rpc_result[ROBOT_CONTROL_TSD10_CHANNELS];
  uint16_t distance_mm[ROBOT_CONTROL_TSD10_CHANNELS];
  uint8_t seq;
  uint8_t flags;
  uint32_t sensor_t_ms;
  uint32_t rx_t_ms;
} RobotControl_Tsd10;

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
} RobotControl_ColorRaw;

typedef struct {
  RobotControl_Tsd10 tsd;
  RobotControl_ColorRaw color;
  int tsd_status;
  int color_status;
} RobotControl_SensorSample;

void RobotControl_Init(void);
void RobotControl_Tick5kHz(void);
void RobotControl_OnCanrpcPublish(uint8_t node, uint8_t topic, const uint8_t *data, uint8_t len);
int RobotControl_ReadTsd10Blocking(RobotControl_Tsd10 *out, uint32_t timeout_ms);
int RobotControl_ReadColorRawBlocking(RobotControl_ColorRaw *out, uint32_t timeout_ms);
int RobotControl_ReadSensorSampleBlocking(RobotControl_SensorSample *out, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_CONTROL_H_INCLUDED */
