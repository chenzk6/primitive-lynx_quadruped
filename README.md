# primitive-lynx_quadruped

Robonix chassis primitive for the DeepRobotics Lynx S10 ("山猫") wheeled
quadruped. Mirrors `primitive-deep_robotics-lite3_quadruped`, but drives the S10
through its **native ROS 2 `drdds` topics** instead of a UDP→ROS2 transfer
bridge:

| S10 topic | type | direction |
|-----------|------|-----------|
| `/MOTION_STATE` | `drdds/msg/MotionState` | out (stand/sit/RL) |
| `/MOTION_INFO`  | `drdds/msg/MotionInfo`  | in (velocities + state @20 Hz) |
| `/NAV_CMD`      | `drdds/msg/NavCmd`      | out (x/y/yaw velocity) |
| `/GAIT`         | `drdds/msg/Gait`        | out (agile nav gait) |
| `/ODOM`         | `nav_msgs/Odometry`     | in (fused pose, re-stamped to `odom` frame) |

Exposed Robonix capabilities: `chassis/driver`, `chassis/move`,
`chassis/twist_in`, `chassis/odom`, `chassis/posture`, `chassis/get_posture`.

## Build & run

```sh
# build (codegen only; drdds is prebuilt on the Jetson)
rbnx codegen -p . --mcp

# run (sources /opt/ros/humble and the prebuilt drdds install)
bash scripts/start.sh
```

The `drdds` message package is prebuilt at `/home/nvidia/drdds_build_ws`;
override its install prefix with `LYNX_DRDDS_INSTALL` if it lives elsewhere.

See `CAPABILITY.md` for the tool contract and the required stand → RL sequence
before movement.
