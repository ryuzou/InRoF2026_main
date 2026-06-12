#ifndef ROBOT_CAN_H
#define ROBOT_CAN_H

#include <stdint.h>

#define NODE_MASTER  0x01u
#define NODE_SENSOR  0x20u
#define NODE_SERVO   0x30u

#define CMD_PING          0x00u
#define CMD_SYSTEM_START  0x01u
#define CMD_SERVO_OPEN_CLOSE  0x10u
#define CMD_POSE_STAGE_X_MM    0x30u
#define CMD_POSE_STAGE_Y_MM    0x31u
#define CMD_POSE_STAGE_H_MRAD  0x32u
#define CMD_POSE_SET_CURRENT   0x33u
#define CMD_POSE_SET_ANCHOR    0x35u

#endif /* ROBOT_CAN_H */
