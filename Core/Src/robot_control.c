#include "robot_control.h"

#include "algorithm.h"
#include "board.h"
#include "canrpc.h"
#include "main.h"

#include <math.h>
#include <string.h>

#define ROBOT_CONTROL_PI  3.14159265358979323846f

#ifndef ROBOT_CONTROL_DT_S
#define ROBOT_CONTROL_DT_S  0.0002f
#endif

#ifndef ROBOT_CONTROL_D_FWD_MIN_MM
#define ROBOT_CONTROL_D_FWD_MIN_MM  5.0f
#endif

#ifndef ROBOT_CONTROL_LAMBDA_DEFAULT_INV_MM
#define ROBOT_CONTROL_LAMBDA_DEFAULT_INV_MM  0.010f
#endif

#ifndef ROBOT_CONTROL_TH_ALLOW_RAD
#define ROBOT_CONTROL_TH_ALLOW_RAD  0.30f
#endif

#ifndef ROBOT_CONTROL_ALPHA
#define ROBOT_CONTROL_ALPHA  0.80f
#endif

#ifndef ROBOT_CONTROL_LAT_EPS_MM
#define ROBOT_CONTROL_LAT_EPS_MM  1.0f
#endif

#ifndef ROBOT_CONTROL_TH_FAIL_RAD
#define ROBOT_CONTROL_TH_FAIL_RAD  0.60f
#endif

#ifndef ROBOT_CONTROL_V_MAX_MM_S
#define ROBOT_CONTROL_V_MAX_MM_S  250.0f
#endif

#ifndef ROBOT_CONTROL_A_MAX_MM_S2
#define ROBOT_CONTROL_A_MAX_MM_S2  500.0f
#endif

#ifndef ROBOT_CONTROL_W_GO_MAX_RAD_S
#define ROBOT_CONTROL_W_GO_MAX_RAD_S  0.25f
#endif

#ifndef ROBOT_CONTROL_W_TURN_MAX_RAD_S
#define ROBOT_CONTROL_W_TURN_MAX_RAD_S  0.80f
#endif

#ifndef ROBOT_CONTROL_W_TURN_MIN_RAD_S
#define ROBOT_CONTROL_W_TURN_MIN_RAD_S  0.08f
#endif

#ifndef ROBOT_CONTROL_TURN_A_MAX_RAD_S2
#define ROBOT_CONTROL_TURN_A_MAX_RAD_S2  2.00f
#endif

#ifndef ROBOT_CONTROL_K_TH
#define ROBOT_CONTROL_K_TH  4.0f
#endif

#ifndef ROBOT_CONTROL_TH_TOL_RAD
#define ROBOT_CONTROL_TH_TOL_RAD  0.035f
#endif

#ifndef ROBOT_CONTROL_POS_TOL_MM
#define ROBOT_CONTROL_POS_TOL_MM  5.0f
#endif

#ifndef ROBOT_CONTROL_LAT_TOL_MM
#define ROBOT_CONTROL_LAT_TOL_MM  10.0f
#endif

#ifndef ROBOT_CONTROL_POSE_STALE_MS
#define ROBOT_CONTROL_POSE_STALE_MS  500u
#endif

#ifndef ROBOT_CONTROL_MOVE_TIMEOUT_MARGIN_MS
#define ROBOT_CONTROL_MOVE_TIMEOUT_MARGIN_MS  2000u
#endif

#ifndef ROBOT_CONTROL_MOVE_TIMEOUT_MS_PER_MM
#define ROBOT_CONTROL_MOVE_TIMEOUT_MS_PER_MM  20.0f
#endif

#ifndef ROBOT_CONTROL_TURN_TIMEOUT_MS
#define ROBOT_CONTROL_TURN_TIMEOUT_MS  8000u
#endif

typedef enum {
  ROBOT_CONTROL_STATE_IDLE = 0,
  ROBOT_CONTROL_STATE_GO,
  ROBOT_CONTROL_STATE_TURN_TO_LINE,
  ROBOT_CONTROL_STATE_TURN_ONLY,
} RobotControl_State;

typedef struct {
  float ax;
  float ay;
  float ux;
  float uy;
  float direction;
  float th_line;
  float target_th;
  float len;
  float k1;
  float k2;
} RobotControl_Command;

static volatile Robot_Pose2D g_sensor_pose;
static volatile RobotControl_Command g_command;
static volatile RobotControl_State g_state = ROBOT_CONTROL_STATE_IDLE;
static volatile RobotControl_CommandResult g_command_result = ROBOT_CONTROL_COMMAND_OK;
static volatile float g_linear_velocity_mm_s = 0.0f;
static volatile float g_angular_velocity_rad_s = 0.0f;
static volatile uint32_t g_command_start_ms = 0u;
static volatile uint32_t g_command_timeout_ms = 0u;
static int32_t g_last_left_mm_s = INT32_MIN;
static int32_t g_last_right_mm_s = INT32_MIN;

static uint32_t RobotControl_EnterCritical(void);
static void RobotControl_ExitCritical(uint32_t primask);
static Robot_Pose2D RobotControl_PoseSnapshotLocal(void);
static bool RobotControl_PoseIsFresh(const Robot_Pose2D *pose, uint32_t now_ms);
static bool RobotControl_BuildMoveCommand(
    const Robot_Pose2D *pose,
    float start_x_mm,
    float start_y_mm,
    float end_x_mm,
    float end_y_mm,
    RobotControl_Command *out
);
static uint32_t RobotControl_MoveTimeoutMs(float len_mm);
static RobotControl_Command RobotControl_CommandSnapshot(void);
static void RobotControl_ControlStep(void);
static float RobotControl_TurnVelocityForError(float th_error);
static void RobotControl_ApplyVelocity(float linear_mm_s, float angular_rad_s);
static void RobotControl_FinishCommand(RobotControl_CommandResult result);
static void RobotControl_AbortCommand(RobotControl_CommandResult result);
static float RobotControl_AbsFloat(float value);
static float RobotControl_MinFloat(float a, float b);
static float RobotControl_MaxFloat(float a, float b);
static float RobotControl_ClampFloat(float value, float min_value, float max_value);
static float RobotControl_WrapPi(float angle_rad);
static float RobotControl_HeadingUnitX(float heading_rad);
static float RobotControl_HeadingUnitY(float heading_rad);
static int32_t RobotControl_RoundToI32(float value);

void RobotControl_Init(void)
{
  uint32_t primask = RobotControl_EnterCritical();

  memset((void *)&g_sensor_pose, 0, sizeof(g_sensor_pose));
  memset((void *)&g_command, 0, sizeof(g_command));
  g_state = ROBOT_CONTROL_STATE_IDLE;
  g_command_result = ROBOT_CONTROL_COMMAND_OK;
  g_linear_velocity_mm_s = 0.0f;
  g_angular_velocity_rad_s = 0.0f;
  g_command_start_ms = 0u;
  g_command_timeout_ms = 0u;

  RobotControl_ExitCritical(primask);

  g_last_left_mm_s = INT32_MIN;
  g_last_right_mm_s = INT32_MIN;
  Algorithm_RobotStop();
}

void RobotControl_Tick5kHz(void)
{
  canrpc_poll();
  RobotControl_ControlStep();
}

void RobotControl_UpdatePose2D(
    uint8_t seq,
    uint32_t sensor_t_ms,
    int32_t x_mm,
    int32_t y_mm,
    float h_rad,
    uint16_t status_flags
)
{
  uint32_t primask = RobotControl_EnterCritical();

  g_sensor_pose.seq = seq;
  g_sensor_pose.sensor_t_ms = sensor_t_ms;
  g_sensor_pose.rx_t_ms = board_millis();
  g_sensor_pose.x_mm = x_mm;
  g_sensor_pose.y_mm = y_mm;
  g_sensor_pose.h_rad = RobotControl_WrapPi(h_rad);
  g_sensor_pose.status_flags = status_flags;
  g_sensor_pose.valid = true;

  RobotControl_ExitCritical(primask);
}

bool RobotControl_GetPoseSnapshot(Robot_Pose2D *out)
{
  if (out == NULL) {
    return false;
  }

  *out = RobotControl_PoseSnapshotLocal();
  return out->valid;
}

RobotControl_CommandResult RobotControl_IssueMoveSegment_mm(
    float start_x_mm,
    float start_y_mm,
    float end_x_mm,
    float end_y_mm
)
{
  if (g_state != ROBOT_CONTROL_STATE_IDLE) {
    return ROBOT_CONTROL_COMMAND_BUSY;
  }

  Robot_Pose2D pose = RobotControl_PoseSnapshotLocal();
  if (!RobotControl_PoseIsFresh(&pose, board_millis())) {
    return ROBOT_CONTROL_COMMAND_NO_POSE;
  }

  RobotControl_Command next_command;
  if (!RobotControl_BuildMoveCommand(
      &pose,
      start_x_mm,
      start_y_mm,
      end_x_mm,
      end_y_mm,
      &next_command
  )) {
    return ROBOT_CONTROL_COMMAND_REJECTED;
  }

  uint32_t primask = RobotControl_EnterCritical();
  if (g_state != ROBOT_CONTROL_STATE_IDLE) {
    RobotControl_ExitCritical(primask);
    return ROBOT_CONTROL_COMMAND_BUSY;
  }

  g_command = next_command;
  g_linear_velocity_mm_s = 0.0f;
  g_angular_velocity_rad_s = 0.0f;
  g_command_start_ms = board_millis();
  g_command_timeout_ms = RobotControl_MoveTimeoutMs(next_command.len);
  g_command_result = ROBOT_CONTROL_COMMAND_RUNNING;
  g_state = ROBOT_CONTROL_STATE_GO;
  RobotControl_ExitCritical(primask);

  return ROBOT_CONTROL_COMMAND_OK;
}

RobotControl_CommandResult RobotControl_IssueTurnTo_rad(float target_heading_rad)
{
  if (g_state != ROBOT_CONTROL_STATE_IDLE) {
    return ROBOT_CONTROL_COMMAND_BUSY;
  }

  Robot_Pose2D pose = RobotControl_PoseSnapshotLocal();
  if (!RobotControl_PoseIsFresh(&pose, board_millis())) {
    return ROBOT_CONTROL_COMMAND_NO_POSE;
  }

  RobotControl_Command next_command;
  memset(&next_command, 0, sizeof(next_command));
  next_command.target_th = RobotControl_WrapPi(target_heading_rad);

  if (RobotControl_AbsFloat(RobotControl_WrapPi(next_command.target_th - pose.h_rad))
      < ROBOT_CONTROL_TH_TOL_RAD) {
    g_command_result = ROBOT_CONTROL_COMMAND_OK;
    return ROBOT_CONTROL_COMMAND_OK;
  }

  uint32_t primask = RobotControl_EnterCritical();
  if (g_state != ROBOT_CONTROL_STATE_IDLE) {
    RobotControl_ExitCritical(primask);
    return ROBOT_CONTROL_COMMAND_BUSY;
  }

  g_command = next_command;
  g_linear_velocity_mm_s = 0.0f;
  g_angular_velocity_rad_s = 0.0f;
  g_command_start_ms = board_millis();
  g_command_timeout_ms = ROBOT_CONTROL_TURN_TIMEOUT_MS;
  g_command_result = ROBOT_CONTROL_COMMAND_RUNNING;
  g_state = ROBOT_CONTROL_STATE_TURN_ONLY;
  RobotControl_ExitCritical(primask);

  return ROBOT_CONTROL_COMMAND_OK;
}

RobotControl_CommandResult RobotControl_IssueTurnTo_deg(float target_heading_deg)
{
  return RobotControl_IssueTurnTo_rad(target_heading_deg * ROBOT_CONTROL_PI / 180.0f);
}

bool RobotControl_IsCommandComplete(void)
{
  return g_state == ROBOT_CONTROL_STATE_IDLE;
}

RobotControl_CommandResult RobotControl_GetCommandResult(void)
{
  return g_command_result;
}

RobotControl_CommandResult RobotControl_WaitForCommandComplete(uint32_t timeout_ms)
{
  uint32_t start_ms = board_millis();

  while (!RobotControl_IsCommandComplete()) {
    if ((timeout_ms != ROBOT_CONTROL_WAIT_FOREVER_MS)
        && ((uint32_t)(board_millis() - start_ms) > timeout_ms)) {
      RobotControl_AbortCommand(ROBOT_CONTROL_COMMAND_TIMEOUT);
      return ROBOT_CONTROL_COMMAND_TIMEOUT;
    }

    board_idle();
  }

  return RobotControl_GetCommandResult();
}

void RobotControl_Stop(void)
{
  RobotControl_AbortCommand(ROBOT_CONTROL_COMMAND_FAILED);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7) {
    RobotControl_Tick5kHz();
  }
}

static uint32_t RobotControl_EnterCritical(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void RobotControl_ExitCritical(uint32_t primask)
{
  if (primask == 0u) {
    __enable_irq();
  }
}

static Robot_Pose2D RobotControl_PoseSnapshotLocal(void)
{
  Robot_Pose2D pose;
  uint32_t primask = RobotControl_EnterCritical();

  pose.valid = g_sensor_pose.valid;
  pose.seq = g_sensor_pose.seq;
  pose.sensor_t_ms = g_sensor_pose.sensor_t_ms;
  pose.rx_t_ms = g_sensor_pose.rx_t_ms;
  pose.x_mm = g_sensor_pose.x_mm;
  pose.y_mm = g_sensor_pose.y_mm;
  pose.h_rad = g_sensor_pose.h_rad;
  pose.status_flags = g_sensor_pose.status_flags;

  RobotControl_ExitCritical(primask);
  return pose;
}

static bool RobotControl_PoseIsFresh(const Robot_Pose2D *pose, uint32_t now_ms)
{
  if ((pose == NULL) || !pose->valid) {
    return false;
  }

  if (ROBOT_CONTROL_POSE_STALE_MS == 0u) {
    return true;
  }

  return (uint32_t)(now_ms - pose->rx_t_ms) <= ROBOT_CONTROL_POSE_STALE_MS;
}

static bool RobotControl_BuildMoveCommand(
    const Robot_Pose2D *pose,
    float start_x_mm,
    float start_y_mm,
    float end_x_mm,
    float end_y_mm,
    RobotControl_Command *out
)
{
  if ((pose == NULL) || (out == NULL)) {
    return false;
  }

  (void)start_x_mm;
  (void)start_y_mm;

  float hx = RobotControl_HeadingUnitX(pose->h_rad);
  float hy = RobotControl_HeadingUnitY(pose->h_rad);
  float dx = end_x_mm - (float)pose->x_mm;
  float dy = end_y_mm - (float)pose->y_mm;
  float d_fwd = dx * hx + dy * hy;
  float direction = (d_fwd >= 0.0f) ? 1.0f : -1.0f;
  float len = RobotControl_AbsFloat(d_fwd);
  float ux = hx * direction;
  float uy = hy * direction;
  float d_lat = dx * uy - dy * ux;

  if (len < ROBOT_CONTROL_D_FWD_MIN_MM) {
    return false;
  }

  float e0 = RobotControl_AbsFloat(d_lat);
  float lam_lo = 5.0f / (ROBOT_CONTROL_ALPHA * len);
  float lam = RobotControl_MaxFloat(ROBOT_CONTROL_LAMBDA_DEFAULT_INV_MM, lam_lo);
  if (e0 > ROBOT_CONTROL_LAT_EPS_MM) {
    float lam_hi = (2.7f * ROBOT_CONTROL_TH_ALLOW_RAD) / e0;
    if (lam_lo > lam_hi) {
      return false;
    }
    lam = RobotControl_ClampFloat(ROBOT_CONTROL_LAMBDA_DEFAULT_INV_MM, lam_lo, lam_hi);
  }

  memset(out, 0, sizeof(*out));
  out->ux = ux;
  out->uy = uy;
  out->direction = direction;
  out->th_line = pose->h_rad;
  out->target_th = out->th_line;
  out->len = len;
  out->ax = end_x_mm - len * ux;
  out->ay = end_y_mm - len * uy;
  out->k1 = lam * lam;
  out->k2 = 2.0f * lam;

  return true;
}

static uint32_t RobotControl_MoveTimeoutMs(float len_mm)
{
  if (len_mm < 0.0f) {
    len_mm = -len_mm;
  }

  return ROBOT_CONTROL_MOVE_TIMEOUT_MARGIN_MS
      + (uint32_t)(len_mm * ROBOT_CONTROL_MOVE_TIMEOUT_MS_PER_MM + 0.5f);
}

static RobotControl_Command RobotControl_CommandSnapshot(void)
{
  RobotControl_Command command;

  command.ax = g_command.ax;
  command.ay = g_command.ay;
  command.ux = g_command.ux;
  command.uy = g_command.uy;
  command.direction = g_command.direction;
  command.th_line = g_command.th_line;
  command.target_th = g_command.target_th;
  command.len = g_command.len;
  command.k1 = g_command.k1;
  command.k2 = g_command.k2;

  return command;
}

static void RobotControl_ControlStep(void)
{
  RobotControl_State state = g_state;
  if (state == ROBOT_CONTROL_STATE_IDLE) {
    RobotControl_ApplyVelocity(0.0f, 0.0f);
    return;
  }

  uint32_t now_ms = board_millis();
  if ((g_command_timeout_ms != 0u)
      && ((uint32_t)(now_ms - g_command_start_ms) > g_command_timeout_ms)) {
    RobotControl_FinishCommand(ROBOT_CONTROL_COMMAND_TIMEOUT);
    RobotControl_ApplyVelocity(0.0f, 0.0f);
    return;
  }

  Robot_Pose2D pose = RobotControl_PoseSnapshotLocal();
  if (!RobotControl_PoseIsFresh(&pose, now_ms)) {
    RobotControl_FinishCommand(ROBOT_CONTROL_COMMAND_FAILED);
    RobotControl_ApplyVelocity(0.0f, 0.0f);
    return;
  }

  RobotControl_Command command = RobotControl_CommandSnapshot();
  float v = g_linear_velocity_mm_s;
  float w = 0.0f;

  if (state == ROBOT_CONTROL_STATE_GO) {
    float px = (float)pose.x_mm - command.ax;
    float py = (float)pose.y_mm - command.ay;
    float s = px * command.ux + py * command.uy;
    float e = px * command.uy - py * command.ux;
    float th_error = RobotControl_WrapPi(pose.h_rad - command.th_line);
    float d_rem = command.len - s;
    float speed = RobotControl_AbsFloat(v);
    float stop_dist = (speed * speed) / (2.0f * ROBOT_CONTROL_A_MAX_MM_S2);

    if (d_rem <= stop_dist) {
      speed = RobotControl_MaxFloat(
          speed - ROBOT_CONTROL_A_MAX_MM_S2 * ROBOT_CONTROL_DT_S,
          0.0f
      );
    } else {
      speed = RobotControl_MinFloat(
          speed + ROBOT_CONTROL_A_MAX_MM_S2 * ROBOT_CONTROL_DT_S,
          ROBOT_CONTROL_V_MAX_MM_S
      );
    }
    v = speed * command.direction;

    w = RobotControl_ClampFloat(
        v * (-command.k1 * e - command.k2 * th_error),
        -ROBOT_CONTROL_W_GO_MAX_RAD_S,
        ROBOT_CONTROL_W_GO_MAX_RAD_S
    );

    if (RobotControl_AbsFloat(th_error) > ROBOT_CONTROL_TH_FAIL_RAD) {
      v = 0.0f;
      w = 0.0f;
      g_angular_velocity_rad_s = 0.0f;
      state = ROBOT_CONTROL_STATE_TURN_TO_LINE;
    }

    if ((d_rem < ROBOT_CONTROL_POS_TOL_MM) || (s > command.len)) {
      v = 0.0f;
      w = 0.0f;
      RobotControl_FinishCommand(
          (RobotControl_AbsFloat(e) < ROBOT_CONTROL_LAT_TOL_MM)
              ? ROBOT_CONTROL_COMMAND_OK
              : ROBOT_CONTROL_COMMAND_FAILED
      );
      RobotControl_ApplyVelocity(v, w);
      return;
    }
  } else if (state == ROBOT_CONTROL_STATE_TURN_TO_LINE) {
    float th_error = RobotControl_WrapPi(pose.h_rad - command.th_line);
    w = RobotControl_TurnVelocityForError(-th_error);

    if (RobotControl_AbsFloat(th_error) < ROBOT_CONTROL_TH_TOL_RAD) {
      w = 0.0f;
      g_angular_velocity_rad_s = 0.0f;
      state = ROBOT_CONTROL_STATE_GO;
    }
  } else if (state == ROBOT_CONTROL_STATE_TURN_ONLY) {
    float th_error = RobotControl_WrapPi(command.target_th - pose.h_rad);
    w = RobotControl_TurnVelocityForError(th_error);

    if (RobotControl_AbsFloat(th_error) < ROBOT_CONTROL_TH_TOL_RAD) {
      w = 0.0f;
      g_angular_velocity_rad_s = 0.0f;
      RobotControl_FinishCommand(ROBOT_CONTROL_COMMAND_OK);
      RobotControl_ApplyVelocity(0.0f, 0.0f);
      return;
    }
  }

  g_linear_velocity_mm_s = v;
  g_angular_velocity_rad_s = w;
  g_state = state;
  RobotControl_ApplyVelocity(v, w);
}

static float RobotControl_TurnVelocityForError(float th_error)
{
  float abs_error = RobotControl_AbsFloat(th_error);
  if (abs_error < ROBOT_CONTROL_TH_TOL_RAD) {
    return 0.0f;
  }

  float target_abs_w = ROBOT_CONTROL_K_TH * abs_error;
  float stop_limited_w = sqrtf(2.0f * ROBOT_CONTROL_TURN_A_MAX_RAD_S2 * abs_error);

  target_abs_w = RobotControl_MinFloat(target_abs_w, stop_limited_w);
  target_abs_w = RobotControl_MinFloat(target_abs_w, ROBOT_CONTROL_W_TURN_MAX_RAD_S);
  target_abs_w = RobotControl_MaxFloat(target_abs_w, ROBOT_CONTROL_W_TURN_MIN_RAD_S);

  float target_w = (th_error > 0.0f) ? target_abs_w : -target_abs_w;
  float current_w = g_angular_velocity_rad_s;
  float max_delta = ROBOT_CONTROL_TURN_A_MAX_RAD_S2 * ROBOT_CONTROL_DT_S;
  float delta = RobotControl_ClampFloat(target_w - current_w, -max_delta, max_delta);

  return current_w + delta;
}

static void RobotControl_ApplyVelocity(float linear_mm_s, float angular_rad_s)
{
  float half_track_mm = ALGORITHM_ROBOT_TRACK_WIDTH_MM * 0.5f;
  int32_t left_mm_s = RobotControl_RoundToI32(linear_mm_s + angular_rad_s * half_track_mm);
  int32_t right_mm_s = RobotControl_RoundToI32(linear_mm_s - angular_rad_s * half_track_mm);

  if ((left_mm_s == g_last_left_mm_s) && (right_mm_s == g_last_right_mm_s)) {
    return;
  }

  Algorithm_RobotSetWheelSpeed_mm_s(left_mm_s, right_mm_s);
  g_last_left_mm_s = left_mm_s;
  g_last_right_mm_s = right_mm_s;
}

static void RobotControl_FinishCommand(RobotControl_CommandResult result)
{
  g_linear_velocity_mm_s = 0.0f;
  g_angular_velocity_rad_s = 0.0f;
  g_command_timeout_ms = 0u;
  g_command_result = result;
  g_state = ROBOT_CONTROL_STATE_IDLE;
}

static void RobotControl_AbortCommand(RobotControl_CommandResult result)
{
  uint32_t primask = RobotControl_EnterCritical();

  RobotControl_FinishCommand(result);

  RobotControl_ExitCritical(primask);
  RobotControl_ApplyVelocity(0.0f, 0.0f);
}

static float RobotControl_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float RobotControl_MinFloat(float a, float b)
{
  return (a < b) ? a : b;
}

static float RobotControl_MaxFloat(float a, float b)
{
  return (a > b) ? a : b;
}

static float RobotControl_ClampFloat(float value, float min_value, float max_value)
{
  if (value < min_value) {
    return min_value;
  }

  if (value > max_value) {
    return max_value;
  }

  return value;
}

static float RobotControl_WrapPi(float angle_rad)
{
  while (angle_rad > ROBOT_CONTROL_PI) {
    angle_rad -= 2.0f * ROBOT_CONTROL_PI;
  }

  while (angle_rad <= -ROBOT_CONTROL_PI) {
    angle_rad += 2.0f * ROBOT_CONTROL_PI;
  }

  return angle_rad;
}

static float RobotControl_HeadingUnitX(float heading_rad)
{
  return sinf(heading_rad);
}

static float RobotControl_HeadingUnitY(float heading_rad)
{
  return cosf(heading_rad);
}

static int32_t RobotControl_RoundToI32(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}
