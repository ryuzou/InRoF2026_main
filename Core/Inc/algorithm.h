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

uint32_t Algorithm_RobotWheelSpeedToStepHz_mm_s(uint32_t speed_mm_s);
void Algorithm_RobotSetWheelSpeed_mm_s(int32_t left_mm_s, int32_t right_mm_s);
void Algorithm_RobotSetVelocity(float linear_mm_s, float angular_rad_s);
void Algorithm_RobotDriveStraight_mm_s(int32_t speed_mm_s);
void Algorithm_RobotTurnInPlace_rad_s(float angular_rad_s);
void Algorithm_RobotStop(void);

#endif /* ALGORITHM_H */
