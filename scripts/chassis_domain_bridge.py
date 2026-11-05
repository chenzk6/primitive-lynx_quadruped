#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""chassis_domain_bridge — bridge the S10's native drdds chassis topics between
the robot's DDS domain and the NX's private ROS 2 domain.

Why this exists
---------------
The S10 motion controller publishes its native topics (the ``drdds`` message
package) on the robot DDS domain (``LYNX_ROBOT_DOMAIN_ID``, default 0) with an
older Fast DDS (RTPS 2.2). The NX runs ROS 2 Humble (Fast DDS 2.6, RTPS 2.3).
Running the whole NX stack on the robot domain makes the NX's RTPS 2.3 discovery
choke on the robot's ``ParticipantEntitiesInfo`` — flooding every node with
``Fast CDR exception`` / ``Bad alloc`` and SIGKILL-ing the C++ nav2 nodes. So the
NX lives on a private domain (``ROS_DOMAIN_ID``, set by the deployment manifest)
and this bridge forwards only the chassis topics across the boundary.

The drdds DATA topics themselves deserialize fine across the version gap (only
the discovery metadata is incompatible), so a lightweight rclpy bridge on the
robot domain is enough — the C++ stack that actually crashes stays isolated on
the private domain.

Directions (from the NX's point of view)
---------------------------------------
  in : robot domain -> private domain   (/MOTION_INFO, /BATTERY_DATA)
  out: private domain -> robot domain   (/MOTION_STATE, /NAV_CMD, /GAIT)

All drdds topics are RELIABLE; the bridge matches that on both sides.
"""
from __future__ import annotations

import os
import signal
import sys
import threading

import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from drdds.msg import BatteryData, Gait, MotionInfo, MotionState, NavCmd

ROBOT_DOMAIN = int(os.environ.get("LYNX_ROBOT_DOMAIN_ID", "0"))
PRIVATE_DOMAIN = int(os.environ.get("ROS_DOMAIN_ID", "42"))

QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# (topic, type, direction) — "in" = robot->NX, "out" = NX->robot
BRIDGED = (
    ("/MOTION_INFO", MotionInfo, "in"),
    ("/BATTERY_DATA", BatteryData, "in"),
    ("/MOTION_STATE", MotionState, "out"),
    ("/NAV_CMD", NavCmd, "out"),
    ("/GAIT", Gait, "out"),
)


def _forward(pub):
    def callback(msg):
        pub.publish(msg)

    return callback


def main() -> int:
    robot_ctx = Context()
    rclpy.init(context=robot_ctx, domain_id=ROBOT_DOMAIN)
    private_ctx = Context()
    rclpy.init(context=private_ctx, domain_id=PRIVATE_DOMAIN)

    robot_node = Node("chassis_bridge_robot", context=robot_ctx)
    private_node = Node("chassis_bridge_private", context=private_ctx)

    for topic, msg_type, direction in BRIDGED:
        sub_node, pub_node = (
            (robot_node, private_node) if direction == "in" else (private_node, robot_node)
        )
        pub = pub_node.create_publisher(msg_type, topic, QOS)
        sub_node.create_subscription(msg_type, topic, _forward(pub), QOS)

    robot_exec = MultiThreadedExecutor(context=robot_ctx)
    robot_exec.add_node(robot_node)
    private_exec = MultiThreadedExecutor(context=private_ctx)
    private_exec.add_node(private_node)

    print(
        f"[chassis_bridge] robot domain={ROBOT_DOMAIN} <-> private domain={PRIVATE_DOMAIN}, "
        f"bridging {len(BRIDGED)} topics",
        flush=True,
    )

    stop = threading.Event()

    def handle_signal(signum, _frame):
        stop.set()
        robot_exec.shutdown()
        private_exec.shutdown()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    for ex in (robot_exec, private_exec):
        threading.Thread(target=ex.spin, daemon=True).start()

    stop.wait()

    robot_node.destroy_node()
    private_node.destroy_node()
    rclpy.shutdown(context=private_ctx)
    rclpy.shutdown(context=robot_ctx)
    return 0


if __name__ == "__main__":
    sys.exit(main())
