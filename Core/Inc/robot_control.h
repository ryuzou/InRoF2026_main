#ifndef ROBOT_CONTROL_H_INCLUDED
#define ROBOT_CONTROL_H_INCLUDED

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bool valid;
  uint8_t seq;
  uint32_t sensor_t_ms;
  uint32_t rx_t_ms;
  int32_t x_mm;
  int32_t y_mm;
  float h_rad;
  uint16_t status_flags;
} Robot_Pose2D;

typedef enum {
  ROBOT_CONTROL_COMMAND_OK = 0,
  ROBOT_CONTROL_COMMAND_RUNNING,
  ROBOT_CONTROL_COMMAND_BUSY,
  ROBOT_CONTROL_COMMAND_NO_POSE,
  ROBOT_CONTROL_COMMAND_REJECTED,
  ROBOT_CONTROL_COMMAND_FAILED,
  ROBOT_CONTROL_COMMAND_TIMEOUT,
} RobotControl_CommandResult;

#define ROBOT_CONTROL_WAIT_FOREVER_MS  0xFFFFFFFFu

void RobotControl_Init(void);
void RobotControl_Tick5kHz(void);
void RobotControl_UpdatePose2D(
    uint8_t seq,
    uint32_t sensor_t_ms,
    int32_t x_mm,
    int32_t y_mm,
    float h_rad,
    uint16_t status_flags
);
bool RobotControl_GetPoseSnapshot(Robot_Pose2D *out);
RobotControl_CommandResult RobotControl_IssueMoveSegment_mm(
    float start_x_mm,
    float start_y_mm,
    float end_x_mm,
    float end_y_mm
);
RobotControl_CommandResult RobotControl_IssueMoveToPose_mm_deg(
    float target_x_mm,
    float target_y_mm,
    float target_heading_deg
);
RobotControl_CommandResult RobotControl_IssueTurnTo_rad(float target_heading_rad);
RobotControl_CommandResult RobotControl_IssueTurnTo_deg(float target_heading_deg);
bool RobotControl_IsCommandComplete(void);
RobotControl_CommandResult RobotControl_GetCommandResult(void);
RobotControl_CommandResult RobotControl_WaitForCommandComplete(uint32_t timeout_ms);
void RobotControl_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_CONTROL_H_INCLUDED */
