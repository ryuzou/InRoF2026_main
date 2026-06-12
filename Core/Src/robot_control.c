#include "robot_control.h"

#include "canrpc.h"
#include "main.h"

void RobotControl_Init(void)
{
}

void RobotControl_Tick5kHz(void)
{
  canrpc_poll();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7) {
    RobotControl_Tick5kHz();
  }
}
