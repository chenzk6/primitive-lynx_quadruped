#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""lynx_quadruped — DeepRobotics Lynx S10 wheeled-quadruped chassis primitive.

Owns ``robonix/primitive/chassis/*`` for the S10, mirroring ``lite3_quadruped``.

Unlike the Lite3 (which needs a UDP→ROS2 ``transfer`` bridge), the S10 exposes
its own ROS 2 topics natively through the custom ``drdds`` message package:

    /MOTION_STATE  (out) drdds/msg/MotionState — state 1=stand 4=sit 17=RL nav
    /MOTION_INFO   (in)  drdds/msg/MotionInfo  — velocities + state + gait @20 Hz
    /NAV_CMD       (out) drdds/msg/NavCmd      — x_vel/y_vel/yaw_vel (RL mode only)
    /GAIT          (out) drdds/msg/Gait        — 0x3002 agile nav gait
    /ODOM          (in)  nav_msgs/Odometry     — fused pose, forwarded as odom

The primitive bridges these to the standard Robonix chassis surface:

    twist_in   (geometry_msgs/Twist)  -> /NAV_CMD
    odom       (nav_msgs/Odometry)    <- /ODOM (re-stamped to odom frame)
    posture / get_posture / move      -> /MOTION_STATE + /NAV_CMD

The S10 already publishes its fused pose on ``/ODOM`` (standard
``nav_msgs/Odometry``, frame ``map``); the primitive forwards it to the
declared ``odom`` capability topic, re-stamping ``frame_id`` from the robot's
LIO ``map`` frame to ``odom_frame`` (default ``odom``) so downstream treats it
as continuous ``odom -> base_link`` odometry rather than a second ``map``
frame. The pose is only meaningful once the robot is localised (it reads
garbage while idle). ``/MOTION_INFO`` yaw rate is still
integrated for the internal yaw used by ``rotate_deg``. ``/NAV_CMD`` is only
honoured in 导航模式 — RL控制(17) + agile nav gait (0x3002) + 使用模式=导航(1) —
so under Robonix control the robot stays in 导航模式: it is switched on init and
re-asserted by ``posture()``, and ``move()`` checks state 17 + gait before moving.
"""
from __future__ import annotations

import json
import logging
import math
import os
import socket
import struct
import threading
import time

from rclpy.clock import Clock
from robonix_api import Primitive, Ok, Err, Deferred

logging.basicConfig(
    level=os.environ.get("LYNX_LOG_LEVEL", "INFO"),
    format="[lynx_quadruped] %(message)s",
)
log = logging.getLogger("lynx_quadruped")

lynx_quadruped = Primitive(id="lynx_quadruped", namespace="robonix/primitive/chassis")

# Generated codegen artifacts (paths wired up by robonix_api at import time).
from quadruped_mcp import (  # noqa: E402
    SetPosture_Request,
    SetPosture_Response,
    GetPosture_Request,
    GetPosture_Response,
)
from chassis_mcp import (  # noqa: E402
    ExecuteMoveCommand_Request,
    ExecuteMoveCommand_Response,
)
from std_msgs_mcp import String  # noqa: E402

# S10 native messages, from the prebuilt drdds package on the Jetson.
from drdds.msg import MotionState, MotionInfo, NavCmd, Gait  # noqa: E402

# ── shared state (guarded by _state_cv) ─────────────────────────────────────
_state_cv = threading.Condition()
_motion_lock = threading.Lock()  # serialises move() bursts

_motion_state: int | None = None  # latest drdds MotionState.state
_motion_stamp = 0.0               # monotonic time of the last MotionInfo
_gait: int | None = None          # latest drdds GaitValue.gait (0x1001 std / 0x3002 nav)

_odom_yaw = 0.0                   # integrated yaw (from vel_yaw), for rotate_deg
_last_info_t: float | None = None  # previous sample time, for dt

# publishers / config filled in on_init
nav_cmd_pub = None
motion_state_pub = None
gait_pub = None
odom_pub = None

base_frame = "base_link"
odom_frame = "odom"

_speed_mps = 0.2
_ang_speed_rps = 0.6
_default_dur = 1.0
_rotate_tolerance_deg = 3.0
_posture_wait_s = 8.0
_nav_gait = 0x3002  # 平地敏捷步态, recommended for navigation

# 本体监控协议 (使用模式切换): the S10's 使用模式 (常规0/导航1/辅助2) is not on
# any ROS 2 topic; /NAV_CMD only works in 导航模式, so posture() switches it here.
_usage_mode_ip = "10.21.33.103"
_usage_mode_port = 30004          # plain UDP (robotserve encryption disabled); TCP alt 30003
_usage_mode_transport = "udp"
_nav_usage_mode = 1               # 使用模式: 1 = 导航模式 (required for /NAV_CMD)


# ── posture / state tables ──────────────────────────────────────────────────
POSTURE_STATES = {
    "stand": 1, "站立": 1,
    "sit": 4, "趴下": 4, "坐": 4,
    "rl": 17, "rl控制": 17, "nav": 17, "导航": 17,
    "damping": 2, "软急停": 2,
    "zero": 5, "标零": 5,
}

STATE_NAMES = {
    0: "idle",
    1: "standing",
    2: "damping",
    3: "boot_damping",
    4: "sitting",
    5: "zeroing",
    17: "rl_control",
    0x100: "damped_sitting",
}


# ── ROS callbacks ───────────────────────────────────────────────────────────
def _on_motion_info(msg) -> None:
    """Track robot state + gait, and integrate yaw rate for rotate_deg."""
    global _motion_state, _motion_stamp, _odom_yaw, _last_info_t, _gait
    d = msg.data
    now = time.monotonic()
    with _state_cv:
        if _last_info_t is not None:
            dt = max(0.0, now - _last_info_t)
            _odom_yaw += float(d.vel_yaw) * dt
        _last_info_t = now
        _motion_state = int(d.motion_state.state)
        _gait = int(d.gait_state.gait)
        _motion_stamp = now
        _state_cv.notify_all()


def _on_native_odom(msg) -> None:
    """Forward the S10's own /ODOM (nav_msgs/Odometry) to the odom capability.

    The S10 already fuses its pose and publishes it on /ODOM stamped in its own
    LIO ``map`` frame. Downstream (the mapping service's map_to_odom_bridge)
    needs this as the continuous ``odom -> base_link`` navigation odometry, so
    we re-stamp the frame_id to ``odom_frame`` instead of passing the robot's
    ``map`` frame through. The pose is only meaningful once the robot is
    localised (it reads garbage while idle). The S10 leaves child_frame_id
    empty, so we fill it with base_frame for tf consumers.
    """
    if odom_pub is None:
        return
    msg.header.frame_id = odom_frame
    # Re-stamp to the HOST clock. The S10's /ODOM carries the robot's own
    # clock, which drifts ~0.3-0.5 s behind this host. map_to_odom_bridge
    # time-syncs this stream against RTAB-Map's ICP odometry with
    # max_sync_error_s=0.10 s, so a robot-clock stamp makes every sync fail
    # ("no chassis odom sample close enough to the ICP timestamp").
    msg.header.stamp = Clock().now().to_msg()
    if not msg.child_frame_id:
        msg.child_frame_id = base_frame
    odom_pub.publish(msg)


def _on_twist_in(msg) -> None:
    """Forward a standard Twist velocity command to the S10 /NAV_CMD topic."""
    if nav_cmd_pub is None:
        return
    cmd = NavCmd()
    cmd.data.x_vel = float(msg.linear.x)
    cmd.data.y_vel = float(msg.linear.y)
    cmd.data.yaw_vel = float(msg.angular.z)
    nav_cmd_pub.publish(cmd)


# ── posture helpers ─────────────────────────────────────────────────────────
def _send_motion_state(state: int) -> None:
    """Publish a /MOTION_STATE transition a few times (the topic is transient)."""
    msg = MotionState()
    msg.data.state = state
    for _ in range(3):
        motion_state_pub.publish(msg)
        time.sleep(0.2)


def _wait_for_state(target: int, timeout: float) -> int | None:
    deadline = time.monotonic() + timeout
    with _state_cv:
        while time.monotonic() < deadline:
            if _motion_state == target:
                return target
            _state_cv.wait(timeout=min(0.2, deadline - time.monotonic()))
    return _motion_state


def _wait_for_gait(target: int, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    with _state_cv:
        while time.monotonic() < deadline:
            if _gait == target:
                return True
            _state_cv.wait(timeout=min(0.2, deadline - time.monotonic()))
    return _gait == target


def _publish_nav_gait() -> None:
    """Switch to the agile nav gait (0x3002 平地敏捷) — the navigation motion
    mode that /NAV_CMD requires. Published a few times like /MOTION_STATE, and
    only after the robot is already in RL控制(17)."""
    if gait_pub is None or not _nav_gait:
        return
    g = Gait()
    g.data.gait = _nav_gait
    for _ in range(3):
        gait_pub.publish(g)
        time.sleep(0.2)


# ── 本体监控协议 (usage-mode switch) ──────────────────────────────────────
# The S10 使用模式 (常规0/导航1/辅助2) is not exposed on any ROS 2 topic — it is
# switched over the 本体监控协议. Frame = 16-byte header + JSON ASDU (see the S10
# dev guide §1.1.5; reference impl in robot-deep_robotics-lynx_s10/.sdk/). A
# heartbeat first registers this client (the robot rejects commands from an
# unmatched client), then the mode-switch command, then its ErrorCode response.
_MONITOR_SYNC = b"\xeb\x91\xeb\x90"
_MONITOR_TYPE_MODE_SWITCH = 0x00100002
_MONITOR_CMD_MODE_SWITCH = 0x00500002
_MONITOR_TYPE_HEARTBEAT = 0x00100064
_MONITOR_CMD_HEARTBEAT = 0x00000005


def _monitor_frame(obj: dict, msg_id: int) -> bytes:
    payload = json.dumps(obj, ensure_ascii=False).encode("utf-8")
    header = _MONITOR_SYNC
    header += struct.pack("<H", len(payload))   # ASDU length, little-endian
    header += struct.pack("<H", msg_id)          # 报文 ID, little-endian
    header += bytes([0x01, 0x00, 0x01])          # JSON | 包编号 | 版本
    header += b"\x00" * 5                        # 预留
    return header + payload


def _switch_usage_mode(mode: int) -> tuple[bool, str]:
    """Switch the S10 使用模式 to `mode` (0常规/1导航/2辅助) over the 本体监控协议.

    One-shot over a fresh socket: heartbeat, mode-switch, then read frames until
    the command response (Items.ErrorCode) arrives. Returns (ok, message).
    """
    if not _usage_mode_ip:
        return False, "usage-mode switch disabled (no usage_mode_ip)"
    now = time.strftime("%Y-%m-%d %H:%M:%S")
    hb = _monitor_frame({
        "PatrolDevice": {"Type": _MONITOR_TYPE_HEARTBEAT, "Command": _MONITOR_CMD_HEARTBEAT,
                         "Time": now, "Items": {}}}, 0)
    cmd = _monitor_frame({
        "PatrolDevice": {"Type": _MONITOR_TYPE_MODE_SWITCH, "Command": _MONITOR_CMD_MODE_SWITCH,
                         "Time": now, "Items": {"Mode": mode}}}, 1)

    sock = None
    try:
        if _usage_mode_transport == "tcp":
            sock = socket.create_connection((_usage_mode_ip, _usage_mode_port), timeout=3.0)
            sock.settimeout(1.0)
            sock.sendall(hb)
            sock.sendall(cmd)
        else:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(1.0)
            dst = (_usage_mode_ip, _usage_mode_port)
            sock.sendto(hb, dst)
            sock.sendto(cmd, dst)

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                data = sock.recv(4096)
            except (socket.timeout, OSError):
                continue
            if len(data) < 16 or data[:4] != _MONITOR_SYNC:
                continue
            try:
                pd = json.loads(data[16:16 + struct.unpack("<H", data[4:6])[0]]
                                .decode("utf-8")).get("PatrolDevice", {})
            except Exception:
                continue
            items = pd.get("Items", {}) or {}
            if "ErrorCode" in items:
                ec = items.get("ErrorCode", -1)
                if ec == 0:
                    return True, f"usage mode={mode} ok"
                return False, (f"usage mode switch failed "
                               f"(ErrorCode=0x{ec:X} {items.get('ErrorMessage', '')})")
        return False, f"no response from {_usage_mode_ip}:{_usage_mode_port}"
    except OSError as e:
        return False, f"usage-mode switch failed: {e}"
    finally:
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass


# ── posture control (MCP) ───────────────────────────────────────────────────
@lynx_quadruped.mcp(
    "robonix/primitive/chassis/posture",
    description=(
        'Switch the Lynx S10 motion state. posture_name: "stand"(站立, ends in '
        'RL控制), "sit"(趴下), "rl"(RL控制), "damping"(软急停), "zero"(标零). '
        'move only works after stand/rl.'
    ),
)
def set_posture(req: SetPosture_Request) -> SetPosture_Response:
    """Set the Lynx S10 to a named motion state.

    posture_name -> drdds MotionState.state:
      "stand"  (站立)   -> 1, then immediately 17. 站立(1) is only a transient
                           (标准运动模式, a half-crouch); the operational
                           standing state is RL控制(17). The dev guide requires
                           switching to RL控制 right after standing.
      "sit"    (趴下)   -> 4
      "rl"     (RL控制) -> 17   (also sets the agile nav gait 0x3002)
      "damping" (关节阻尼/软急停) -> 2
      "zero"   (标零)   -> 5

    /NAV_CMD (used by move) only works in 导航模式: RL控制(17) + nav gait
    0x3002 + 使用模式=导航(1). "stand"/"rl" end there — both switch the gait,
    then the 使用模式 over the 本体监控协议."""
    raw = (req.posture_name or "").strip()
    state = POSTURE_STATES.get(raw) or POSTURE_STATES.get(raw.lower())
    if state is None:
        return SetPosture_Response(
            success=False,
            message=(
                f"unknown posture {req.posture_name!r}; valid: "
                "stand(站立), sit(趴下), rl(RL控制), damping(软急停), zero(标零)"
            ),
        )

    if state == 1:
        # 站立(1) only stands the robot up into 标准运动模式 (a half-crouch).
        # The real standing/operational state is RL控制(17); chain straight to it.
        _send_motion_state(1)
        _wait_for_state(1, _posture_wait_s)
        time.sleep(0.3)
        _send_motion_state(17)
        target = 17
    else:
        _send_motion_state(state)
        target = state

    observed = _wait_for_state(target, _posture_wait_s)

    gait_ok = True
    mode_ok = True
    mode_msg = ""
    if observed == 17:
        # /NAV_CMD (move) is ignored unless the robot is in RL控制(17) + an agile
        # nav gait AND its 使用模式 is 导航(1). Switch the gait once 17 is actually
        # reached (so /GAIT is not lost during the transition), then switch usage
        # mode over the 本体监控协议.
        _publish_nav_gait()
        gait_ok = _wait_for_gait(_nav_gait, _posture_wait_s)
        if gait_ok:
            mode_ok, mode_msg = _switch_usage_mode(_nav_usage_mode)

    if observed == target and gait_ok and mode_ok:
        return SetPosture_Response(
            success=True,
            message=(
                f"{raw} ok (state={target} {STATE_NAMES.get(target, '')}"
                f", gait={hex(_gait) if _gait else 'unknown'}"
                f"{', ' + mode_msg if mode_msg else ''})"
            ),
        )
    if observed != target:
        return SetPosture_Response(
            success=False,
            message=(
                f"sent {raw} (state={target}); not reached within {_posture_wait_s}s "
                f"(observed={observed} {STATE_NAMES.get(observed, '')})"
            ),
        )
    if not gait_ok:
        return SetPosture_Response(
            success=False,
            message=(
                f"reached RL控制(17) but nav gait {hex(_nav_gait)} not confirmed "
                f"(gait={hex(_gait) if _gait else 'unknown'})"
            ),
        )
    return SetPosture_Response(
        success=False,
        message=(
            f"reached RL控制(17) + nav gait but {mode_msg}; "
            "move (/NAV_CMD) may be ignored"
        ),
    )


@lynx_quadruped.mcp(
    "robonix/primitive/chassis/get_posture",
    description=(
        'Read the S10 current motion state (read-only). posture: standing/'
        'sitting/rl_control/damping/boot_damping/zeroing/idle/unknown; '
        'available=false means the stream is stale.'
    ),
)
def get_posture(req: GetPosture_Request) -> GetPosture_Response:
    """Read the robot's CURRENT motion state from the /MOTION_INFO stream.

    Read-only: never sends a command. Use this to confirm a posture change
    instead of re-sending set_posture. posture is one of 'standing' (站立),
    'sitting' (趴下), 'rl_control' (导航/RL控制), 'damping', 'boot_damping',
    'zeroing', 'idle', or 'unknown'. If available is false the S10 stream is
    stale — tell the user posture acquisition failed instead of guessing."""
    global _motion_state, _motion_stamp
    available = _motion_state is not None and (time.monotonic() - _motion_stamp) < 1.0
    if not available:
        return GetPosture_Response(posture="unknown", basic_state=-1, available=False)
    return GetPosture_Response(
        posture=STATE_NAMES.get(_motion_state, "unknown"),
        basic_state=_motion_state,
        available=True,
    )


# ── move (MCP) ─────────────────────────────────────────────────────────────
def _move_response(obj) -> ExecuteMoveCommand_Response:
    return ExecuteMoveCommand_Response(
        status=String(data=json.dumps(obj))
    )


def _publish_navcmd_for_duration(cmd, duration: float) -> None:
    """Publish /NAV_CMD at 10 Hz for the duration, then always send a stop."""
    try:
        deadline = time.monotonic() + max(0.1, duration)
        while time.monotonic() < deadline:
            nav_cmd_pub.publish(cmd)
            time.sleep(0.1)
    finally:
        nav_cmd_pub.publish(NavCmd())


def _wrap_angle(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def _rotate_closed_loop(target_deg: float, max_speed: float):
    """Rotate in place, closing the loop on the integrated yaw."""
    target = math.radians(target_deg)
    tolerance = math.radians(_rotate_tolerance_deg)
    timeout = max(5.0, abs(target) / max(0.1, max_speed) * 2.5 + 2.0)
    started = time.monotonic()
    with _state_cv:
        if _motion_state is None or started - _motion_stamp > 1.0:
            _state_cv.wait(timeout=1.0)
        if _motion_state is None or time.monotonic() - _motion_stamp > 1.0:
            return False, 0.0, 0.0, "motion stream unavailable"
        previous = _odom_yaw

    accumulated = 0.0
    command = NavCmd()
    try:
        while time.monotonic() - started < timeout:
            with _state_cv:
                _state_cv.wait(timeout=0.1)
                if time.monotonic() - _motion_stamp > 1.0:
                    return False, math.degrees(accumulated), time.monotonic() - started, "motion stream became stale"
                current = _odom_yaw
            accumulated += _wrap_angle(current - previous)
            previous = current
            remaining = target - accumulated
            if abs(remaining) <= tolerance:
                return True, math.degrees(accumulated), time.monotonic() - started, ""
            speed = min(max_speed, max(0.15, abs(remaining) * 1.2))
            command.data.yaw_vel = math.copysign(speed, remaining)
            nav_cmd_pub.publish(command)
        return False, math.degrees(accumulated), time.monotonic() - started, "rotation timeout"
    finally:
        nav_cmd_pub.publish(NavCmd())


@lynx_quadruped.mcp(
    "robonix/primitive/chassis/move",
    description=(
        "Move or rotate the Lynx S10 base with a single bounded burst, then "
        "stop. The robot must be in RL nav mode first (posture('stand') already "
        "ends there; confirm with get_posture)."
    ),
)
def move(req: ExecuteMoveCommand_Request) -> ExecuteMoveCommand_Response:
    """Move or rotate the Lynx S10 base with one bounded burst, then stop.

    `command` is an object whose fields are used in this priority order:

    - forward_m (float, metres): signed forward/backward distance, moved at the
      configured speed. Example: {"command": {"forward_m": 0.1}}.
    - rotate_deg (float, degrees): signed in-place yaw rotation; positive is
      CCW/left, negative is CW/right. Example: {"command": {"rotate_deg": -30}}.
    - linear_x/y/z + angular_x/y/z (m/s, rad/s) with duration_sec (s): raw
      velocity burst. Only used when forward_m and rotate_deg are both 0.

    Precondition: the robot must be in RL nav mode — state 17 plus the agile
    nav gait (0x3002/0x3003) plus 使用模式=导航(1). posture('stand')/posture('rl')
    set all three; move re-asserts the 使用模式 over the 本体监控协议 and refuses
    to publish /NAV_CMD if that switch fails.
    """
    if nav_cmd_pub is None:
        return _move_response({"status": "error", "error": "ROS2 not initialized"})

    # /NAV_CMD only takes effect in navigation mode, which is three things
    # together: state 17 + agile nav gait (0x3002) + 使用模式=导航(1). The first
    # two come from /MOTION_INFO; the 使用模式 is not on any ROS 2 topic, so it is
    # re-asserted over the 本体监控协议 (idempotent) before every move.
    if _motion_state != 17 or (_gait != _nav_gait):
        return _move_response({
            "status": "error",
            "error": (
                f"robot not in RL nav mode (state={_motion_state}, "
                f"gait={hex(_gait) if _gait else 'unknown'}); "
                "call posture('stand') or posture('rl') first"
            ),
        })
    ok, mode_msg = _switch_usage_mode(_nav_usage_mode)
    if not ok:
        return _move_response({
            "status": "error",
            "error": (
                f"not in navigation mode (usage-mode switch failed: {mode_msg}); "
                "call posture('stand') or posture('rl') first"
            ),
        })

    msg = req.command
    with _motion_lock:
        cmd = NavCmd()
        actual_rotate_deg = 0.0
        if float(getattr(msg, "forward_m", 0.0)) != 0.0:
            forward_m = float(msg.forward_m)
            sign = 1.0 if forward_m > 0 else -1.0
            cmd.data.x_vel = sign * _speed_mps
            duration = abs(forward_m) / _speed_mps
            mode = "forward_m"
            _publish_navcmd_for_duration(cmd, duration)
        elif float(getattr(msg, "rotate_deg", 0.0)) != 0.0:
            rotate_deg = float(msg.rotate_deg)
            ok, actual_rotate_deg, duration, error = _rotate_closed_loop(
                rotate_deg, _ang_speed_rps
            )
            if not ok:
                return _move_response({
                    "status": "error",
                    "mode": "rotate_deg",
                    "error": error,
                    "requested_rotate_deg": rotate_deg,
                    "actual_rotate_deg": actual_rotate_deg,
                })
            mode = "rotate_deg"
        else:
            cmd.data.x_vel = float(getattr(msg, "linear_x", 0.0))
            cmd.data.y_vel = float(getattr(msg, "linear_y", 0.0))
            cmd.data.yaw_vel = float(getattr(msg, "angular_z", 0.0))
            duration = float(getattr(msg, "duration_sec", 0.0))
            if duration <= 0.0:
                duration = _default_dur
            mode = "velocity"
            _publish_navcmd_for_duration(cmd, duration)

    return _move_response({
        "status": "done",
        "mode": mode,
        "duration_sec": duration,
        "x_vel": cmd.data.x_vel,
        "y_vel": cmd.data.y_vel,
        "yaw_vel": cmd.data.yaw_vel,
        "actual_rotate_deg": actual_rotate_deg,
    })


# ── lifecycle ───────────────────────────────────────────────────────────────
@lynx_quadruped.on_init
def init(cfg):
    """INACTIVE: open the drdds publishers/subscriptions, await the S10 stream."""
    global nav_cmd_pub, motion_state_pub, gait_pub, odom_pub
    global base_frame, odom_frame
    global _speed_mps, _ang_speed_rps, _default_dur, _rotate_tolerance_deg, _posture_wait_s, _nav_gait
    global _usage_mode_ip, _usage_mode_port, _usage_mode_transport, _nav_usage_mode

    motion_state_topic = cfg.get("motion_state_topic", "/MOTION_STATE")
    motion_info_topic = cfg.get("motion_info_topic", "/MOTION_INFO")
    nav_cmd_topic = cfg.get("nav_cmd_topic", "/NAV_CMD")
    gait_topic = cfg.get("gait_topic", "/GAIT")
    twist_in_topic = cfg.get("twist_in_topic", "/cmd_vel")
    odom_topic = cfg.get("odom_topic", "/odom")
    odom_source_topic = cfg.get("odom_source_topic", "/ODOM")
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 30.0))

    base_frame = str(cfg.get("base_frame", "base_link"))
    odom_frame = str(cfg.get("odom_frame", "odom"))
    _speed_mps = float(cfg.get("speed_mps", os.environ.get("LYNX_SPEED_MPS", "0.2")))
    _ang_speed_rps = float(cfg.get("ang_speed_rps", os.environ.get("LYNX_ANG_SPEED_RPS", "0.6")))
    _default_dur = float(cfg.get("default_duration_sec", os.environ.get("LYNX_DEFAULT_DURATION_SEC", "1.0")))
    _rotate_tolerance_deg = float(cfg.get("rotate_tolerance_deg", os.environ.get("LYNX_ROTATE_TOLERANCE_DEG", "3.0")))
    _posture_wait_s = float(cfg.get("posture_wait_s", 8.0))
    _nav_gait = int(cfg.get("nav_gait", 0x3002))

    # 本体监控协议 (使用模式切换) — plain UDP 30004 / TCP 30003 (encryption off).
    _usage_mode_ip = str(cfg.get("usage_mode_ip", os.environ.get("LYNX_USAGE_MODE_IP", "10.21.33.103")))
    _usage_mode_port = int(cfg.get("usage_mode_port", os.environ.get("LYNX_USAGE_MODE_PORT", "30004")))
    _usage_mode_transport = str(cfg.get("usage_mode_transport", "udp")).lower()
    _nav_usage_mode = int(cfg.get("usage_mode", 1))

    # Publishers for the S10's native drdds topics (internal, not declared).
    nav_cmd_pub = lynx_quadruped.create_publisher(
        "_nav_cmd", topic=nav_cmd_topic, msg_type=NavCmd, qos="reliable", declare=False)
    motion_state_pub = lynx_quadruped.create_publisher(
        "_motion_state", topic=motion_state_topic, msg_type=MotionState, qos="reliable", declare=False)
    gait_pub = lynx_quadruped.create_publisher(
        "_gait", topic=gait_topic, msg_type=Gait, qos="reliable", declare=False)

    # Subscribe to the S10 motion state/velocity stream (reliable: the S10
    # publishes it reliable, and a best-effort subscriber would not match).
    lynx_quadruped.create_subscription(
        "_motion_info", topic=motion_info_topic, msg_type=MotionInfo,
        callback=_on_motion_info, qos="reliable", declare=False)

    # twist_in bridge: standard Twist -> drdds NavCmd.
    from geometry_msgs.msg import Twist  # type: ignore
    lynx_quadruped.create_subscription(
        "robonix/primitive/chassis/twist_in", topic=twist_in_topic, msg_type=Twist,
        callback=_on_twist_in, qos="reliable", declare=False)
    lynx_quadruped.declare_ros2_topic(
        "robonix/primitive/chassis/twist_in", twist_in_topic, qos="reliable")

    # odom: forward the S10's native /ODOM (nav_msgs/Odometry) to the declared
    # odom capability topic. The S10 publishes /ODOM reliable (like its drdds
    # topics), so subscribe reliable.
    from nav_msgs.msg import Odometry  # type: ignore
    odom_pub = lynx_quadruped.create_publisher(
        "robonix/primitive/chassis/odom", topic=odom_topic, msg_type=Odometry,
        qos="reliable", declare=False)
    lynx_quadruped.declare_ros2_topic(
        "robonix/primitive/chassis/odom", odom_topic, qos="reliable")
    lynx_quadruped.create_subscription(
        "_odom_source", topic=odom_source_topic, msg_type=Odometry,
        callback=_on_native_odom, qos="reliable", declare=False)

    # Await the S10's motion stream before reporting ready.
    with _state_cv:
        if _motion_state is None:
            _state_cv.wait(timeout=sentinel_timeout)
    if _motion_state is None:
        return Deferred(f"no {motion_info_topic} stream from the S10 within {sentinel_timeout}s")

    # Under Robonix control the S10 stays in 导航模式 (使用模式=1) the whole time
    # — /NAV_CMD (move) needs it and nothing else switches it. Set it now;
    # posture() re-asserts it after each stand/rl in case a transition reset it.
    ok, msg = _switch_usage_mode(_nav_usage_mode)
    if ok:
        log.info("usage mode=%d (%s)", _nav_usage_mode, msg)
    else:
        log.warning("initial usage-mode switch not applied: %s", msg)

    log.info(
        "lynx_quadruped ready: state=%s odom=%s<-%s nav_cmd=%s twist_in=%s",
        _motion_state, odom_topic, odom_source_topic, nav_cmd_topic, twist_in_topic,
    )
    return Ok()


@lynx_quadruped.on_shutdown
def shutdown():
    """Send a zero /NAV_CMD so the robot does not keep driving on exit."""
    if nav_cmd_pub is not None:
        try:
            nav_cmd_pub.publish(NavCmd())
        except Exception:  # noqa: BLE001
            pass
    return Ok()


if __name__ == "__main__":
    lynx_quadruped.run()
