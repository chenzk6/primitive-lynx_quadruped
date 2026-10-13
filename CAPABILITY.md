---
description: Lynx S10 底盘控制（移动、站/趴/导航模式姿态读写）
---

# lynx_quadruped — Lynx S10 wheeled-quadruped chassis primitive

Wraps the DeepRobotics Lynx S10 ("山猫") motion host over its native ROS 2
topics (`drdds/msg/*`) and exposes posture control plus burst-style movement
to the pilot. Use this capability whenever the user asks the robot to stand,
sit, enter navigation mode, or move (forward/backward/rotate).

Unlike the Lite3, the S10 has its own internal computer and onboard
navigation; this primitive only drives the chassis. Sensors (lidar/IMU/camera)
and services (mapping/nav2/speech) are out of scope here.

## Interface (3 tools + 2 topics)

### `robonix/primitive/chassis/posture` — set_posture

Set the S10 to a named motion state by publishing `drdds/msg/MotionState` on
`/MOTION_STATE`. Returns `{success, message}`. `success=false` means the target
state was not observed within `posture_wait_s` seconds.

| posture_name | state | description |
|--------------|-------|-------------|
| `"stand"` (站立) | 1 → 17 | stands up (站立, a transient half-crouch), then switches to RL控制 (17), the operational standing state |
| `"sit"` (趴下) | 4 | sitting posture |
| `"rl"` (RL控制) | 17 | enter RL control directly; also sets the agile nav gait (0x3002) |
| `"damping"` (关节阻尼/软急停) | 2 | joint damping / soft emergency stop |
| `"zero"` (标零) | 5 | zero calibration |

> **Before moving:** `/NAV_CMD` (used by `move`) is honoured only in **导航模式**,
> which is three things together: RL控制 (17) + agile nav gait (0x3002) +
> **使用模式 = 导航 (1)**. Under Robonix control the robot stays in 导航模式: the
> primitive switches it on init and re-asserts it in `posture("stand")`/
> `posture("rl")`. The 使用模式 is switched over the 本体监控协议 (plain UDP
> 30004 / TCP 30003, see config). Confirm with `get_posture` (`rl_control`).

### `robonix/primitive/chassis/get_posture` — get_posture

Read the robot's real motion state from `/MOTION_INFO` (read-only). Returns
`{posture, basic_state, available}`:

| field | meaning |
|-------|---------|
| `posture` | `standing` / `sitting` / `rl_control` / `damping` / `boot_damping` / `zeroing` / `idle` / `unknown` |
| `basic_state` | raw drdds `MotionState.state` (1/4/17/…); `-1` when unavailable |
| `available` | `false` if the S10 stream is stale (robot unreachable) |

**Recommended flow:** after `set_posture(...)`, call `get_posture` to confirm.
If `available` is `false`, tell the user posture acquisition failed instead of
guessing.

### `robonix/primitive/chassis/move` — move

Single-shot velocity burst on `/NAV_CMD`, then stop. **Not a continuous
stream.** Requires RL control mode (17).

| field | meaning |
|-------|---------|
| `command.forward_m` | forward distance (m), positive = forward |
| `command.rotate_deg` | in-place yaw rotation (deg), positive = CCW |
| `command.linear_x/y/z`, `angular_x/y/z` | raw velocity (m/s, rad/s) |
| `command.duration_sec` | raw-velocity duration (default 1.0 s) |

Modes (priority): `forward_m` → `rotate_deg` → raw velocity. `forward_m`
drives at `speed_mps` (default 0.2 m/s); `rotate_deg` closes the loop on the
dead-reckoned yaw. Returns JSON `{status, mode, duration_sec, …}` in `status`.

> ⚠️ **Precondition:** the robot must be in 导航模式 — RL控制 (17) **and** the
> agile nav gait (0x3002/0x3003) **and** 使用模式=导航(1). `posture("stand")`/
> `posture("rl")` set all three; `/NAV_CMD` is ignored otherwise. `move`
> re-checks state+gait from `/MOTION_INFO` **and** re-asserts the 使用模式 over
> the 本体监控协议 before publishing `/NAV_CMD`; if either fails it returns
> `{status:"error"}` asking the pilot to run `posture("stand")` or
> `posture("rl")` first.

### `robonix/primitive/chassis/twist_in` — continuous velocity (topic)

A standard `geometry_msgs/Twist` topic (default `/cmd_vel`) bridged to the
S10's `/NAV_CMD`. Future navigation services publish Twist here; each message
is converted to `drdds/NavCmd` (x_vel/y_vel/yaw_vel).

### `robonix/primitive/chassis/odom` — odometry (topic)

A `nav_msgs/Odometry` stream (default `/odom`) forwarded from the
S10's own `/ODOM` topic (a standard `nav_msgs/Odometry`, frame `map`). The S10
fuses its own pose, so no dead-reckoning is done here. `frame_id` is re-stamped
from the robot's LIO `map` frame to `odom_frame` (default `odom`) so consumers
treat it as continuous `odom -> base_link` odometry rather than a second
`map` frame; `child_frame_id` is filled with `base_frame` because the S10
leaves it empty. The pose is only meaningful once the robot is localised —
while idle it reads garbage (huge x/y/z). The source is configurable via
`odom_source_topic`; `/LIO_ODOM` and `/LIO_ODOM_HIGH_FREQUENCY` are
alternatives that only publish while the LIO stack is running.

## Recommended pilot workflow

1. `posture("stand")`, confirm `get_posture` reports `rl_control`.
2. `move(command={"forward_m": 1.0})` / `move(command={"rotate_deg": 90})`.
3. `posture("sit")` when done, confirm `sitting`.

## Config (deploy manifest `config:`)

| key | default | meaning |
|-----|---------|---------|
| `motion_state_topic` | `/MOTION_STATE` | state transition command topic |
| `motion_info_topic` | `/MOTION_INFO` | motion state/velocity stream |
| `nav_cmd_topic` | `/NAV_CMD` | navigation velocity command topic |
| `gait_topic` | `/GAIT` | gait switch topic |
| `twist_in_topic` | `/cmd_vel` | standard Twist input |
| `odom_topic` | `/odom` | forwarded odometry output |
| `odom_source_topic` | `/ODOM` | native S10 odometry topic to forward |
| `odom_frame` | `odom` | `frame_id` stamped on the forwarded odometry |
| `base_frame` | `base_link` | fills the S10's empty child_frame_id |
| `sentinel_timeout_s` | `30.0` | wait for `/MOTION_INFO` on init |
| `speed_mps` | `0.2` | forward/backward speed |
| `ang_speed_rps` | `0.6` | rotation speed |
| `default_duration_sec` | `1.0` | raw-velocity default duration |
| `rotate_tolerance_deg` | `3.0` | closed-loop yaw tolerance |
| `posture_wait_s` | `8.0` | wait for a posture transition |
| `nav_gait` | `12290` | gait set on RL entry (0x3002 平地敏捷) |
| `usage_mode_ip` | `10.21.33.103` | 本体监控协议 host (使用模式切换) |
| `usage_mode_port` | `30004` | plain UDP port (TCP alt `30003`; encryption disabled) |
| `usage_mode_transport` | `udp` | `udp` or `tcp` for the usage-mode switch |
| `usage_mode` | `1` | 使用模式 set on RL entry (1 = 导航模式) |

## What this primitive does NOT do

- It does not correct the S10's odometry (it forwards `/ODOM` as-is, only
  re-stamping the frame name).
- It does not do obstacle avoidance or global path planning.
- It does not expose the S10's lidar/IMU/camera or UDP 本体监控协议.
