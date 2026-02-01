"""Real Vive Tracker Publisher ROS2 Node.

Wraps ViveTrackerManager in a ROS2 node that reads real Vive Tracker
data via PyOpenVR/SteamVR and publishes PoseStamped messages on
the standard tracker topics.

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 -m teleop_system.devices.vive_tracker_pub \\
        --ros-args -p tracker_mapping.right_hand:=LHR-XXXXXXXX \\
                   -p tracker_mapping.left_hand:=LHR-YYYYYYYY \\
                   -p tracker_mapping.waist:=LHR-ZZZZZZZZ
"""

from teleop_system.utils.logger import get_logger

logger = get_logger("vive_tracker_pub")

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped

    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    logger.warning("ROS2 not available — ViveTrackerPub cannot be used")

if _ROS2_AVAILABLE:
    import json

    import numpy as np
    from std_msgs.msg import String

    from teleop_system.interfaces.master_device import Pose6D, TrackerRole
    from teleop_system.utils.ros2_helpers import (
        QoSPreset,
        TopicNames,
        get_qos_profile,
    )

    _ROLE_TO_TOPIC = {
        TrackerRole.RIGHT_HAND: TopicNames.TRACKER_RIGHT_HAND,
        TrackerRole.LEFT_HAND: TopicNames.TRACKER_LEFT_HAND,
        TrackerRole.WAIST: TopicNames.TRACKER_WAIST,
        TrackerRole.RIGHT_FOOT: TopicNames.TRACKER_RIGHT_FOOT,
        TrackerRole.LEFT_FOOT: TopicNames.TRACKER_LEFT_FOOT,
    }

    def _pose6d_to_pose_stamped(pose: Pose6D, clock, frame_id: str = "world") -> PoseStamped:
        """Convert internal Pose6D to geometry_msgs/PoseStamped."""
        msg = PoseStamped()
        msg.header.stamp = clock.now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(pose.position[0])
        msg.pose.position.y = float(pose.position[1])
        msg.pose.position.z = float(pose.position[2])
        msg.pose.orientation.x = float(pose.orientation[0])
        msg.pose.orientation.y = float(pose.orientation[1])
        msg.pose.orientation.z = float(pose.orientation[2])
        msg.pose.orientation.w = float(pose.orientation[3])
        return msg

    class ViveTrackerPub(Node):
        """ROS2 node that reads real Vive Tracker data and publishes PoseStamped.

        Uses ViveTrackerManager to connect to SteamVR, discover trackers,
        and publish their poses at a configurable rate.

        Publishes:
            /master/tracker/right      (PoseStamped)
            /master/tracker/left       (PoseStamped)
            /master/tracker/waist      (PoseStamped)
            /master/tracker/right_foot (PoseStamped)
            /master/tracker/left_foot  (PoseStamped)
        """

        def __init__(self, node_name: str = "vive_tracker_pub"):
            super().__init__(node_name)

            # Parameters
            self.declare_parameter("rate_hz", 100.0)
            self.declare_parameter("tracker_mapping.right_hand", "")
            self.declare_parameter("tracker_mapping.left_hand", "")
            self.declare_parameter("tracker_mapping.waist", "")
            self.declare_parameter("tracker_mapping.right_foot", "")
            self.declare_parameter("tracker_mapping.left_foot", "")
            self.declare_parameter("auto_detect", True)

            rate = self.get_parameter("rate_hz").get_parameter_value().double_value

            # Build tracker serial mapping from parameters
            tracker_mapping = {}
            for role_name in ["right_hand", "left_hand", "waist", "right_foot", "left_foot"]:
                serial = self.get_parameter(
                    f"tracker_mapping.{role_name}"
                ).get_parameter_value().string_value
                if serial:
                    tracker_mapping[role_name] = serial

            auto_detect = self.get_parameter("auto_detect").get_parameter_value().bool_value

            # Initialize ViveTrackerManager
            try:
                from teleop_system.devices.vive_tracker import ViveTrackerManager
            except ImportError:
                self.get_logger().error(
                    "ViveTracker import failed. Is PyOpenVR installed? "
                    "pip install openvr"
                )
                raise RuntimeError("ViveTracker not available")

            self._manager = ViveTrackerManager(
                tracker_mapping=tracker_mapping if tracker_mapping else None,
                auto_detect=auto_detect,
            )

            if not self._manager.initialize():
                self.get_logger().error(
                    "Failed to initialize ViveTrackerManager. "
                    "Is SteamVR running?"
                )
                raise RuntimeError("ViveTrackerManager init failed")

            # Get discovered trackers
            self._trackers = self._manager.get_all_trackers()
            role_names = [r.name for r in self._trackers]
            self.get_logger().info(f"Found {len(self._trackers)} trackers: {role_names}")

            # Create publishers for discovered trackers
            sensor_qos = get_qos_profile(QoSPreset.SENSOR_DATA)
            self._tracker_pubs = {}
            for role in self._trackers:
                topic = _ROLE_TO_TOPIC.get(role)
                if topic:
                    self._tracker_pubs[role] = self.create_publisher(
                        PoseStamped, topic, sensor_qos
                    )
                    self.get_logger().info(f"Publishing {role.name} on {topic}")

            # Calibration offsets
            self._calibration_offsets: dict[TrackerRole, np.ndarray] = {}
            status_qos = get_qos_profile(QoSPreset.STATUS)
            self._offset_sub = self.create_subscription(
                String, TopicNames.CALIBRATION_OFFSETS,
                self._offset_cb, status_qos,
            )

            # Timer
            self._timer = self.create_timer(1.0 / rate, self._publish_all)

            self.get_logger().info(f"ViveTrackerPub started at {rate} Hz")

        def _offset_cb(self, msg: String) -> None:
            """Parse calibration offsets from JSON."""
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            role_map = {
                "right_hand": TrackerRole.RIGHT_HAND,
                "left_hand": TrackerRole.LEFT_HAND,
                "waist": TrackerRole.WAIST,
                "right_foot": TrackerRole.RIGHT_FOOT,
                "left_foot": TrackerRole.LEFT_FOOT,
                "head": TrackerRole.HEAD,
            }
            for name, offset_list in data.items():
                role = role_map.get(name)
                if role:
                    self._calibration_offsets[role] = np.array(offset_list)
            self.get_logger().info(
                f"Calibration offsets received: {list(data.keys())}"
            )

        def _publish_all(self) -> None:
            """Read each tracker and publish its pose."""
            for role, tracker in self._trackers.items():
                if role not in self._tracker_pubs:
                    continue
                pose = tracker.get_pose()
                if not pose.valid:
                    continue
                # Apply calibration offset
                if role in self._calibration_offsets:
                    pose.position = pose.position + self._calibration_offsets[role]
                msg = _pose6d_to_pose_stamped(pose, self.get_clock())
                self._tracker_pubs[role].publish(msg)

        def destroy_node(self):
            self._manager.shutdown()
            super().destroy_node()


def main(args=None):
    """Entry point for the Vive Tracker publisher."""
    if not _ROS2_AVAILABLE:
        raise RuntimeError("ROS2 is not available. Install ros-jazzy-desktop.")

    rclpy.init(args=args)
    node = ViveTrackerPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
