"""Dummy Tracker Publisher ROS2 Node.

Wraps SimulatedTracker instances in a ROS2 node that publishes
PoseStamped messages on the standard tracker topics for testing
without real Vive Tracker hardware.

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 -m teleop_system.simulators.dummy_tracker_pub
"""

from teleop_system.utils.logger import get_logger

logger = get_logger("dummy_tracker_pub")

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped

    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    logger.warning("ROS2 not available — DummyTrackerPub cannot be used")

if _ROS2_AVAILABLE:
    import json

    import numpy as np
    from std_msgs.msg import String

    from teleop_system.interfaces.master_device import Pose6D, TrackerRole
    from teleop_system.simulators.simulated_tracker import SimulatedTracker
    from teleop_system.utils.ros2_helpers import (
        QoSPreset,
        TopicNames,
        get_qos_profile,
    )

    # TrackerRole -> ROS2 topic name
    _ROLE_TO_TOPIC = {
        TrackerRole.RIGHT_HAND: TopicNames.TRACKER_RIGHT_HAND,
        TrackerRole.LEFT_HAND: TopicNames.TRACKER_LEFT_HAND,
        TrackerRole.WAIST: TopicNames.TRACKER_WAIST,
        TrackerRole.RIGHT_FOOT: TopicNames.TRACKER_RIGHT_FOOT,
        TrackerRole.LEFT_FOOT: TopicNames.TRACKER_LEFT_FOOT,
    }

    def pose6d_to_pose_stamped(pose: Pose6D, clock, frame_id: str = "world") -> PoseStamped:
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

    class DummyTrackerPub(Node):
        """ROS2 node that publishes simulated tracker data for testing.

        Creates SimulatedTracker instances for right hand, left hand, waist,
        and optionally feet, and publishes their poses as PoseStamped at
        a configurable rate.

        Publishes:
            /master/tracker/right      (PoseStamped)
            /master/tracker/left       (PoseStamped)
            /master/tracker/waist      (PoseStamped)
            /master/tracker/right_foot (PoseStamped)  [if publish_feet=true]
            /master/tracker/left_foot  (PoseStamped)  [if publish_feet=true]
        """

        def __init__(self, node_name: str = "dummy_tracker_pub"):
            super().__init__(node_name)

            # Parameters
            self.declare_parameter("rate_hz", 100.0)
            self.declare_parameter("amplitude", 0.08)
            self.declare_parameter("frequency", 0.3)
            self.declare_parameter("publish_feet", True)

            rate = self.get_parameter("rate_hz").get_parameter_value().double_value
            amplitude = self.get_parameter("amplitude").get_parameter_value().double_value
            frequency = self.get_parameter("frequency").get_parameter_value().double_value
            publish_feet = self.get_parameter("publish_feet").get_parameter_value().bool_value

            # Create simulated trackers (hands + waist)
            self._trackers = {
                TrackerRole.RIGHT_HAND: SimulatedTracker(
                    role=TrackerRole.RIGHT_HAND,
                    amplitude=amplitude,
                    frequency=frequency,
                    phase_offset=0.0,
                ),
                TrackerRole.LEFT_HAND: SimulatedTracker(
                    role=TrackerRole.LEFT_HAND,
                    amplitude=amplitude,
                    frequency=frequency,
                    phase_offset=np.pi / 2,
                ),
                TrackerRole.WAIST: SimulatedTracker(
                    role=TrackerRole.WAIST,
                    amplitude=amplitude * 0.5,
                    frequency=frequency * 0.5,
                ),
            }

            # Add foot trackers (anti-phase for walking pattern)
            if publish_feet:
                self._trackers[TrackerRole.RIGHT_FOOT] = SimulatedTracker(
                    role=TrackerRole.RIGHT_FOOT,
                    amplitude=5.0,
                    frequency=0.5,
                    phase_offset=0.0,
                )
                self._trackers[TrackerRole.LEFT_FOOT] = SimulatedTracker(
                    role=TrackerRole.LEFT_FOOT,
                    amplitude=5.0,
                    frequency=0.5,
                    phase_offset=np.pi,
                )
            for tracker in self._trackers.values():
                tracker.initialize()

            # Create publishers
            sensor_qos = get_qos_profile(QoSPreset.SENSOR_DATA)
            self._tracker_pubs = {}
            for role in self._trackers:
                topic = _ROLE_TO_TOPIC.get(role)
                if topic:
                    self._tracker_pubs[role] = self.create_publisher(
                        PoseStamped, topic, sensor_qos
                    )

            # Calibration offsets
            self._calibration_offsets: dict[TrackerRole, np.ndarray] = {}
            status_qos = get_qos_profile(QoSPreset.STATUS)
            self._offset_sub = self.create_subscription(
                String, TopicNames.CALIBRATION_OFFSETS,
                self._offset_cb, status_qos,
            )

            # Timer
            self._timer = self.create_timer(1.0 / rate, self._publish_all)

            self.get_logger().info(
                f"DummyTrackerPub started: {len(self._trackers)} trackers at {rate} Hz "
                f"(amplitude={amplitude}, frequency={frequency})"
            )

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
            """Read each tracker and publish its pose as PoseStamped."""
            for role, tracker in self._trackers.items():
                if role not in self._tracker_pubs:
                    continue
                pose = tracker.get_pose()
                if not pose.valid:
                    continue
                # Apply calibration offset
                if role in self._calibration_offsets:
                    pose.position = pose.position + self._calibration_offsets[role]
                msg = pose6d_to_pose_stamped(pose, self.get_clock())
                self._tracker_pubs[role].publish(msg)

        def destroy_node(self):
            for tracker in self._trackers.values():
                tracker.shutdown()
            super().destroy_node()


def main(args=None):
    """Entry point for the dummy tracker publisher."""
    if not _ROS2_AVAILABLE:
        raise RuntimeError("ROS2 is not available. Install ros-jazzy-desktop.")

    rclpy.init(args=args)
    node = DummyTrackerPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
