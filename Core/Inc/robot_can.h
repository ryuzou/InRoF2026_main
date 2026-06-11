#ifndef ROBOT_CAN_H
#define ROBOT_CAN_H

#include <stdint.h>

#define NODE_MASTER  0x01u
#define NODE_SENSOR  0x20u
#define NODE_SERVO   0x30u

#define CMD_PING          0x00u
#define CMD_SYSTEM_START  0x01u

#endif /* ROBOT_CAN_H */
