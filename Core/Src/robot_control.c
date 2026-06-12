#include "robot_control.h"

#include "board.h"
#include "canrpc.h"
#include "main.h"
#include "robot_can.h"
#include <string.h>

#define CMD_TSD_READ             0x10u
#define CMD_COLOR_MEASURE        0x20u

#define TOPIC_COLOR_RAW          0x30u

#define SENSOR_COLOR_ARG(atime, gain, flags) \
  ((int32_t)((uint32_t)(atime) | ((uint32_t)(gain) << 8) | ((uint32_t)(flags) << 16)))

static volatile RobotControl_ColorRaw s_color_raw;

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms);
static bool time_not_before(uint32_t t_ms, uint32_t start_ms);
static uint16_t get_u16_le(const uint8_t *p);
static uint32_t get_u32_le(const uint8_t *p);
static void color_snapshot(RobotControl_ColorRaw *out);

void RobotControl_Init(void)
{
  memset((void *)&s_color_raw, 0, sizeof(s_color_raw));
  s_color_raw.rpc_result = CANRPC_RES_INVALID;
}

void RobotControl_Tick5kHz(void)
{
  canrpc_poll();
}

void RobotControl_OnCanrpcPublish(uint8_t node, uint8_t topic, const uint8_t *data, uint8_t len)
{
  if (node != NODE_SENSOR || data == NULL) {
    return;
  }

  if (topic == TOPIC_COLOR_RAW && len >= 17u) {
    s_color_raw.seq = data[0];
    s_color_raw.rpc_result = CANRPC_OK;
    s_color_raw.sensor_t_ms = get_u32_le(&data[1]);
    s_color_raw.rx_t_ms = HAL_GetTick();
    s_color_raw.clear = get_u16_le(&data[5]);
    s_color_raw.red = get_u16_le(&data[7]);
    s_color_raw.green = get_u16_le(&data[9]);
    s_color_raw.blue = get_u16_le(&data[11]);
    s_color_raw.atime = data[13];
    s_color_raw.gain = data[14];
    s_color_raw.led_on_ms = data[15];
    s_color_raw.flags = data[16];
    s_color_raw.valid = true;
  }
}

int RobotControl_ReadTsd10Blocking(RobotControl_Tsd10 *out, uint32_t timeout_ms)
{
  if (out == NULL) {
    return CANRPC_ERR_PARAM;
  }

  RobotControl_Tsd10 tsd;
  memset(&tsd, 0, sizeof(tsd));
  for (uint8_t ch = 0; ch < ROBOT_CONTROL_TSD10_CHANNELS; ch++) {
    tsd.rpc_result[ch] = CANRPC_RES_INVALID;
  }

  int handles[ROBOT_CONTROL_TSD10_CHANNELS] = {-1, -1, -1};
  uint32_t wait_mask = 0u;

  for (uint8_t ch = 0; ch < ROBOT_CONTROL_TSD10_CHANNELS; ch++) {
    handles[ch] = canrpc_call(NODE_SENSOR, CMD_TSD_READ, (int32_t)ch);
    if (handles[ch] >= 0) {
      wait_mask |= CANRPC_H(handles[ch]);
    } else {
      tsd.rpc_result[ch] = CANRPC_RES_BUSY;
    }
  }

  int rc = (wait_mask != 0u) ? canrpc_wait(wait_mask, timeout_ms) : CANRPC_ERR_PARAM;
  tsd.rx_t_ms = HAL_GetTick();

  for (uint8_t ch = 0; ch < ROBOT_CONTROL_TSD10_CHANNELS; ch++) {
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

int RobotControl_ReadColorRawBlocking(RobotControl_ColorRaw *out, uint32_t timeout_ms)
{
  if (out == NULL) {
    return CANRPC_ERR_PARAM;
  }

  RobotControl_ColorRaw color;
  memset(&color, 0, sizeof(color));
  color.rpc_result = CANRPC_RES_INVALID;

  uint32_t start_ms = board_millis();
  uint32_t deadline_ms = start_ms + timeout_ms;
  int handle = canrpc_call(
      NODE_SENSOR,
      CMD_COLOR_MEASURE,
      SENSOR_COLOR_ARG(
          ROBOT_CONTROL_COLOR_ATIME,
          ROBOT_CONTROL_COLOR_GAIN,
          ROBOT_CONTROL_COLOR_FLAGS
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
    color_snapshot(&color);
    if (color.valid && color.seq == expected_seq && time_not_before(color.rx_t_ms, start_ms)) {
      *out = color;
      return 0;
    }

    if (time_reached(board_millis(), deadline_ms)) {
      color.valid = false;
      color.seq = expected_seq;
      color.rpc_result = CANRPC_OK;
      *out = color;
      return CANRPC_ERR_TIMEOUT;
    }

    board_idle();
  }
}

int RobotControl_ReadSensorSampleBlocking(RobotControl_SensorSample *out, uint32_t timeout_ms)
{
  if (out == NULL) {
    return CANRPC_ERR_PARAM;
  }

  RobotControl_SensorSample sample;
  memset(&sample, 0, sizeof(sample));

  sample.tsd_status = RobotControl_ReadTsd10Blocking(&sample.tsd, timeout_ms);
  sample.color_status = RobotControl_ReadColorRawBlocking(&sample.color, timeout_ms);

  *out = sample;
  return (sample.tsd_status != 0) ? sample.tsd_status : sample.color_status;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7) {
    RobotControl_Tick5kHz();
  }
}

static bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
  return (int32_t)(now_ms - deadline_ms) >= 0;
}

static bool time_not_before(uint32_t t_ms, uint32_t start_ms)
{
  return (int32_t)(t_ms - start_ms) >= 0;
}

static uint16_t get_u16_le(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t get_u32_le(const uint8_t *p)
{
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static void color_snapshot(RobotControl_ColorRaw *out)
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
