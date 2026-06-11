# CANRPC Command Memo

This file tracks the CANRPC command map used by `InRoF2026_main`.
Keep this memo in sync with `Core/Inc/robot_can.h` whenever node IDs or
common command IDs change.

## Source Of Truth

- Common node IDs and common command IDs: `Core/Inc/robot_can.h`
- Sensor-specific command IDs: `InRoF2026_Sensors/Core/Inc/sensor_node_can.h`

## Frame ID

CANRPC uses an 11-bit standard CAN ID:

```c
((type & 0x7) << 8) | (node & 0xff)
```

| Type | Value | Direction |
| --- | ---: | --- |
| CMD | `1` | caller to target node |
| ACK | `2` | target node to caller |
| DONE | `3` | target node to caller |
| PUB | `4` | publisher node to all subscribers |

Current transport is CAN FD with BRS enabled. Maximum frame payload is 64 bytes.

## Common Nodes

Defined in `Core/Inc/robot_can.h`.

| Node | ID | Owner |
| --- | ---: | --- |
| `NODE_MASTER` | `0x01` | `InRoF2026_main` |
| `NODE_SENSOR` | `0x20` | `InRoF2026_Sensors` |
| `NODE_SERVO` | `0x30` | `InRoF2026_Servo` |

## Common Commands

Defined in `Core/Inc/robot_can.h`.

| Command | ID | arg | ret | Notes |
| --- | ---: | ---: | ---: | --- |
| `CMD_PING` | `0x00` | `0` | implementation-defined | Link check |
| `CMD_SYSTEM_START` | `0x01` | `0` | `0` | Sent by Master to Sensor and Servo after `START_SW` |

`CMD_SYSTEM_START` is the shared start notification. The receiver should leave
its waiting or bring-up state and enter the competition-ready state. If the
receiver does not implement this command, CANRPC returns `CANRPC_RES_NO_HANDLER`.
If the receiver is not online, the caller sees `CANRPC_RES_NO_ACK`.

## Master Startup Sequence

`InRoF2026_main` starts CANRPC as `NODE_MASTER`. After `START_SW` is pressed,
it sends `CMD_SYSTEM_START` to both `NODE_SENSOR` and `NODE_SERVO`, then waits
up to `1000 ms` for both DONE responses. Any failure currently enters
`Error_Handler()`.

## Sensor Commands

Existing Sensor command IDs copied from `InRoF2026_Sensors/Core/Inc/sensor_node_can.h`.
Move these into a shared header later if multiple nodes need to include them.

| Command | ID |
| --- | ---: |
| `CMD_TSD_READ` | `0x10` |
| `CMD_TSD_PUBLISH` | `0x11` |
| `CMD_COLOR_MEASURE` | `0x20` |
| `CMD_COLOR_GET_PACKED` | `0x21` |
| `CMD_POSE_STAGE_X_MM` | `0x30` |
| `CMD_POSE_STAGE_Y_MM` | `0x31` |
| `CMD_POSE_STAGE_H_MRAD` | `0x32` |
| `CMD_POSE_COMMIT` | `0x33` |
| `CMD_POSE_RESET_ORIGIN` | `0x34` |
| `CMD_POSE_SET_ANCHOR` | `0x35` |
| `CMD_OTOS_SELF_TEST` | `0x40` |
| `CMD_OTOS_CALIB_IMU` | `0x41` |
| `CMD_OTOS_GET_STATUS` | `0x42` |
| `CMD_SET_POSE_PUB_PERIOD` | `0x50` |
| `CMD_GET_NODE_STATUS` | `0x7f` |
