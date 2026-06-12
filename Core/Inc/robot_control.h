#ifndef ROBOT_CONTROL_H_INCLUDED
#define ROBOT_CONTROL_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

void RobotControl_Init(void);
void RobotControl_Tick5kHz(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_CONTROL_H_INCLUDED */
