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

## Servo Commands

Defined in `Core/Inc/robot_can.h` and implemented by `InRoF2026_Servo`.

| Command | ID | arg | ret | Notes |
| --- | ---: | ---: | ---: | --- |
| `CMD_SERVO_OPEN_CLOSE` | `0x10` | `0` | `0` | Moves the servo from the initial/closed `78 deg` position to `120 deg`, waits `500 ms`, then returns to `78 deg` |

`InRoF2026_Servo` sets the servo to `78 deg` during initialization. The
command handler sends DONE only after the servo has returned to `78 deg`, so a
caller that needs blocking behavior should use `canrpc_call_wait()` or
`canrpc_call_wait_ret()` with a timeout longer than `500 ms`, for example:

```c
int rc = canrpc_call_wait(NODE_SERVO, CMD_SERVO_OPEN_CLOSE, 0, 1000u);
```

## Master Startup Sequence

`InRoF2026_main` starts CANRPC as `NODE_MASTER`. After `START_SW` is pressed,
it sends `CMD_SYSTEM_START` to both `NODE_SENSOR` and `NODE_SERVO`, then waits
up to `1000 ms` for both DONE responses. Any failure currently enters
`Error_Handler()`.

## Sensor Commands

Existing Sensor command IDs copied from `InRoF2026_Sensors/Core/Inc/sensor_node_can.h`.
Move these into a shared header later if multiple nodes need to include them.

| Command | ID | arg | ret | Notes |
| --- | ---: | ---: | ---: | --- |
| `CMD_TSD_READ` | `0x10` | channel `0..2` | distance mm | Reads one TSD channel |
| `CMD_TSD_PUBLISH` | `0x11` | `0` | seq | Publishes `TOPIC_TSD_ALL`; disabled from default autonomous publishing |
| `CMD_COLOR_MEASURE` | `0x20` | `SENSOR_COLOR_ARG(atime,gain,flags)` | seq | Starts LED-on color measurement; publishes `TOPIC_COLOR_RAW` when complete |
| `CMD_POSE_STAGE_X_MM` | `0x30` | x mm | `0` | Stages current-pose x |
| `CMD_POSE_STAGE_Y_MM` | `0x31` | y mm | `0` | Stages current-pose y |
| `CMD_POSE_STAGE_H_MRAD` | `0x32` | heading mrad | `0` | Stages current-pose heading |
| `CMD_POSE_SET_CURRENT` | `0x33` | `0` | `0` | Applies staged x/y/h as the current OTOS pose; `0,0,0` is the origin case |
| `CMD_POSE_SET_ANCHOR` | `0x35` | anchor id | `0` | Applies a compile-time pose anchor |
| `CMD_OTOS_SELF_TEST` | `0x40` | `0` | `1` pass, `0` fail | Non-blocking self-test |
| `CMD_OTOS_CALIB_IMU` | `0x41` | sample count `1..255` | remaining samples | Non-blocking IMU calibration |
| `CMD_OTOS_GET_STATUS` | `0x42` | `0` | status byte | Reads OTOS status |
| `CMD_SET_POSE_PUB_PERIOD` | `0x50` | period ms, `0` disables | `0` | Controls `TOPIC_POSE2D` period |
| `CMD_GET_NODE_STATUS` | `0x7f` | `0` | compact status flags | RPC status read; `TOPIC_NODE_STATUS` is disabled by default |

## Sensor PUB Topics

| Topic | ID | data_len | Publisher | Default |
| --- | ---: | ---: | --- | --- |
| `TOPIC_POSE2D` | `0x10` | 19 | Periodic after `CMD_SYSTEM_START` | Enabled |
| `TOPIC_TSD_ALL` | `0x20` | 12 | Only from `CMD_TSD_PUBLISH` | Disabled |
| `TOPIC_COLOR_RAW` | `0x30` | 17 | Only when `CMD_COLOR_MEASURE` completes | Command-triggered |
| `TOPIC_NODE_STATUS` | `0x7f` | 32 | Periodic node status | Disabled |

`TOPIC_POSE2D` payload:

```text
[0]      seq u8
[1..4]   t_ms u32
[5..8]   x_mm i32
[9..12]  y_mm i32
[13..16] h_mrad i32
[17..18] status_flags u16
```

`TOPIC_COLOR_RAW` is not an autonomous publish. The sensor turns the color LED
on only for `CMD_COLOR_MEASURE`, waits for integration, turns the LED off, then
publishes the raw RGBC result once.
