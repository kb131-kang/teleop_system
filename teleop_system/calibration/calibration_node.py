"""ROS2 node for A-Pose tracker calibration.

Wraps PoseCalibrator in a ROS2 node that:
- Subscribes to all tracker topics to feed pose data
- Provides /teleop/calibrate service (std_srvs/Trigger) to start calibration
- Publishes calibration state on /calibration/state (JSON)
- Publishes computed offsets on /calibration/offsets (JSON, transient local)
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from teleop_system.calibration.pose_calibrator import (
    CalibrationConfig,
    CalibrationState,
    PoseCalibrator,
)
from teleop_system.interfaces.master_device import Pose6D, TrackerRole
from teleop_system.utils.logger import get_logger

logger = get_logger("calibration_node")


def main():
    """Entry point for the calibration node."""
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped, QuaternionStamped
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
    except ImportError:
        logger.error("ROS2 (rclpy) is required for CalibrationNode")
        return

    from teleop_system.utils.ros2_helpers import (
        TopicNames,
        ServiceNames,
        QoSPreset,
        get_qos_profile,
    )

    class CalibrationNode(Node):
        """ROS2 node wrapping PoseCalibrator for A-Pose calibration."""

        def __init__(self, node_name: str = "calibration_node"):
            super().__init__(node_name)

            # Parameters
            self.declare_parameter("config_file", "")

            config_file = (
                self.get_parameter("config_file")
                .get_parameter_value()
                .string_value
            )

            if config_file and Path(config_file).exists():
                config = CalibrationConfig.from_yaml(config_file)
            else:
                config = CalibrationConfig._default()

            self._calibrator = PoseCalibrator(config)

            # Latest poses from trackers
            self._current_poses: dict[TrackerRole, Pose6D] = {}

            # Subscribers for tracker topics
            sensor_qos = get_qos_profile(QoSPreset.SENSOR_DATA)

            self._tracker_subs = {
                TrackerRole.RIGHT_HAND: self.create_subscription(
                    PoseStamped, TopicNames.TRACKER_RIGHT_HAND,
                    lambda msg, r=TrackerRole.RIGHT_HAND: self._tracker_cb(r, msg),
                    sensor_qos,
                ),
                TrackerRole.LEFT_HAND: self.create_subscription(
                    PoseStamped, TopicNames.TRACKER_LEFT_HAND,
                    lambda msg, r=TrackerRole.LEFT_HAND: self._tracker_cb(r, msg),
                    sensor_qos,
                ),
                TrackerRole.WAIST: self.create_subscription(
                    PoseStamped, TopicNames.TRACKER_WAIST,
                    lambda msg, r=TrackerRole.WAIST: self._tracker_cb(r, msg),
                    sensor_qos,
                ),
                TrackerRole.RIGHT_FOOT: self.create_subscription(
                    PoseStamped, TopicNames.TRACKER_RIGHT_FOOT,
                    lambda msg, r=TrackerRole.RIGHT_FOOT: self._tracker_cb(r, msg),
                    sensor_qos,
                ),
                TrackerRole.LEFT_FOOT: self.create_subscription(
                    PoseStamped, TopicNames.TRACKER_LEFT_FOOT,
                    lambda msg, r=TrackerRole.LEFT_FOOT: self._tracker_cb(r, msg),
                    sensor_qos,
                ),
            }

            # HMD subscription (QuaternionStamped → HEAD pose with position=[0,0,0])
            self._hmd_sub = self.create_subscription(
                QuaternionStamped, TopicNames.HMD_ORIENTATION,
                self._hmd_cb, sensor_qos,
            )

            # Calibrate service
            self._calibrate_srv = self.create_service(
                Trigger, ServiceNames.CALIBRATE, self._calibrate_callback,
            )

            # State publisher (10Hz)
            status_qos = get_qos_profile(QoSPreset.STATUS)
            self._state_pub = self.create_publisher(
                String, TopicNames.CALIBRATION_STATE, status_qos,
            )
            self._offset_pub = self.create_publisher(
                String, TopicNames.CALIBRATION_OFFSETS, status_qos,
            )

            # Update timer (100Hz)
            self._timer = self.create_timer(0.01, self._update)
            # State publish timer (10Hz)
            self._state_timer = self.create_timer(0.1, self._publish_state)

            self._last_published_state = None

            self.get_logger().info("Calibration node ready")

        def _tracker_cb(self, role: TrackerRole, msg: PoseStamped) -> None:
            self._current_poses[role] = Pose6D(
                position=np.array([
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                ]),
                orientation=np.array([
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ]),
                valid=True,
            )

        def _hmd_cb(self, msg: QuaternionStamped) -> None:
            self._current_poses[TrackerRole.HEAD] = Pose6D(
                position=np.zeros(3),
                orientation=np.array([
                    msg.quaternion.x,
                    msg.quaternion.y,
                    msg.quaternion.z,
                    msg.quaternion.w,
                ]),
                valid=True,
            )

        def _calibrate_callback(self, request, response):
            success = self._calibrator.start_calibration()
            response.success = success
            if success:
                response.message = "Calibration started"
                self.get_logger().info("Calibration triggered via service")
            else:
                response.message = (
                    f"Cannot start: state={self._calibrator.state.name}"
                )
                self.get_logger().warning(response.message)
            return response

        def _update(self) -> None:
            state = self._calibrator.update(self._current_poses)

            # Publish offsets when calibration completes
            if (state == CalibrationState.CALIBRATED
                    and self._last_published_state != CalibrationState.CALIBRATED):
                self._publish_offsets()

            self._last_published_state = state

        def _publish_state(self) -> None:
            state = self._calibrator.state
            msg = String()
            msg.data = json.dumps({
                "state": state.name,
                "progress": round(self._calibrator.progress, 3),
                "countdown": round(self._calibrator.countdown_remaining, 1),
                "error": self._calibrator.error_message,
            })
            self._state_pub.publish(msg)

        def _publish_offsets(self) -> None:
            offsets = self._calibrator.get_offsets()
            if not offsets:
                return

            role_names = {
                TrackerRole.RIGHT_HAND: "right_hand",
                TrackerRole.LEFT_HAND: "left_hand",
                TrackerRole.WAIST: "waist",
                TrackerRole.RIGHT_FOOT: "right_foot",
                TrackerRole.LEFT_FOOT: "left_foot",
                TrackerRole.HEAD: "head",
            }

            data = {}
            for role, offset in offsets.items():
                name = role_names.get(role, role.name)
                data[name] = offset.tolist()

            msg = String()
            msg.data = json.dumps(data)
            self._offset_pub.publish(msg)
            self.get_logger().info(f"Published calibration offsets: {data}")

    rclpy.init()
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
