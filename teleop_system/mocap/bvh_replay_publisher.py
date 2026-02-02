"""ROS2 node that publishes BVH motion data as tracker/hand/HMD topics.

Drop-in replacement for DummyTrackerPub + DummyGlovePub + DummyHMDPub.
Publishes BVH motion capture data on the same topics with the same
message types, allowing the standard teleop nodes to process it.

Staged start: when auto_start is False (default), the node starts in
READY state — publishing the initial frame (frame 0) repeatedly so
the robot aligns to the BVH starting pose. Call /teleop/start_playback
service to begin advancing through the motion data.
"""

import json
from pathlib import Path

import numpy as np

from teleop_system.interfaces.master_device import TrackerRole
from teleop_system.utils.logger import get_logger

logger = get_logger("bvh_replay_publisher")


def main():
    """Entry point for BVH replay publisher node."""
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped, QuaternionStamped
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
    except ImportError:
        logger.error("ROS2 (rclpy) is required for BVHReplayPub")
        return

    from teleop_system.utils.ros2_helpers import (
        TopicNames, ServiceNames, QoSPreset, get_qos_profile,
    )
    from teleop_system.mocap.bvh_loader import load_bvh_lazy
    from teleop_system.mocap.skeleton_mapper import SkeletonMapper
    from teleop_system.mocap.bvh_tracker_adapter import BVHTrackerAdapter
    from teleop_system.mocap.bvh_hand_adapter import BVHHandAdapter

    class BVHReplayPub(Node):
        """ROS2 node publishing BVH motion data on tracker/hand/HMD topics.

        When auto_start is False (default), the node starts in READY state:
        it publishes the initial frame (frame 0) repeatedly so the robot
        aligns to the BVH starting pose. Call /teleop/start_playback to
        begin advancing through the motion data.

        States:
            READY    — publishing initial frame, waiting for start command
            PLAYING  — advancing through BVH frames
        """

        def __init__(self, node_name: str = "bvh_replay_pub"):
            super().__init__(node_name)

            # Declare parameters
            self.declare_parameter("bvh_file", "")
            self.declare_parameter("rate_hz", 120.0)
            self.declare_parameter("playback_speed", 1.0)
            self.declare_parameter("loop", True)
            self.declare_parameter("scale", 0.056)
            self.declare_parameter("normalize_mode", "relative")
            self.declare_parameter("auto_start", False)

            bvh_file = self.get_parameter("bvh_file").get_parameter_value().string_value
            rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
            playback_speed = self.get_parameter("playback_speed").get_parameter_value().double_value
            loop = self.get_parameter("loop").get_parameter_value().bool_value
            scale = self.get_parameter("scale").get_parameter_value().double_value
            normalize_mode = self.get_parameter("normalize_mode").get_parameter_value().string_value
            auto_start = self.get_parameter("auto_start").get_parameter_value().bool_value

            if not bvh_file:
                self.get_logger().error("No BVH file specified. Set bvh_file parameter.")
                return

            bvh_path = Path(bvh_file)
            if not bvh_path.exists():
                self.get_logger().error(f"BVH file not found: {bvh_path}")
                return

            # Load and map BVH data
            target_joints = [
                "Hips", "Head", "LeftHand", "RightHand",
                "LeftFoot", "RightFoot",
            ]
            bvh_data = load_bvh_lazy(str(bvh_path), scale=scale, joints=target_joints)

            mapper = SkeletonMapper(normalize_mode=normalize_mode)
            self._mapped_motion = mapper.map(bvh_data)

            # Create tracker adapters
            self._trackers = {}
            for role in TrackerRole:
                adapter = BVHTrackerAdapter(
                    role=role,
                    mapped_motion=self._mapped_motion,
                    playback_speed=playback_speed,
                    loop=loop,
                )
                adapter.initialize()
                self._trackers[role] = adapter

            # Create hand adapters
            self._hands = {}
            for side in ["left", "right"]:
                adapter = BVHHandAdapter(
                    side=side,
                    mapped_motion=self._mapped_motion,
                    playback_speed=playback_speed,
                    loop=loop,
                )
                adapter.initialize()
                self._hands[side] = adapter

            # Playback state: READY (paused at frame 0) or PLAYING
            self._playing = False
            if auto_start:
                self._playing = True
            else:
                # Pause all adapters at frame 0
                for adapter in self._trackers.values():
                    adapter.pause()
                for adapter in self._hands.values():
                    adapter.pause()

            # Create publishers
            qos = get_qos_profile(QoSPreset.SENSOR_DATA)

            self._tracker_pubs = {
                TrackerRole.RIGHT_HAND: self.create_publisher(
                    PoseStamped, TopicNames.TRACKER_RIGHT_HAND, qos),
                TrackerRole.LEFT_HAND: self.create_publisher(
                    PoseStamped, TopicNames.TRACKER_LEFT_HAND, qos),
                TrackerRole.WAIST: self.create_publisher(
                    PoseStamped, TopicNames.TRACKER_WAIST, qos),
                TrackerRole.RIGHT_FOOT: self.create_publisher(
                    PoseStamped, TopicNames.TRACKER_RIGHT_FOOT, qos),
                TrackerRole.LEFT_FOOT: self.create_publisher(
                    PoseStamped, TopicNames.TRACKER_LEFT_FOOT, qos),
                TrackerRole.HEAD: self.create_publisher(
                    PoseStamped, TopicNames.TRACKER_HEAD, qos),
            }

            self._hmd_pub = self.create_publisher(
                QuaternionStamped, TopicNames.HMD_ORIENTATION, qos)

            self._hand_pubs = {
                "left": self.create_publisher(
                    JointState, TopicNames.HAND_LEFT_JOINTS, qos),
                "right": self.create_publisher(
                    JointState, TopicNames.HAND_RIGHT_JOINTS, qos),
            }

            # Calibration offsets
            self._calibration_offsets: dict = {}
            status_qos = get_qos_profile(QoSPreset.STATUS)
            self._offset_sub = self.create_subscription(
                String, TopicNames.CALIBRATION_OFFSETS,
                self._offset_cb, status_qos,
            )

            # Playback state publisher (transient local so GUI gets current state)
            self._playback_state_pub = self.create_publisher(
                String, TopicNames.PLAYBACK_STATE, status_qos,
            )

            # Start playback service
            self._start_srv = self.create_service(
                Trigger, ServiceNames.START_PLAYBACK,
                self._start_playback_cb,
            )

            # Timer
            timer_period = 1.0 / rate_hz
            self._timer = self.create_timer(timer_period, self._publish_all)

            # Publish state at 2Hz
            self._state_timer = self.create_timer(0.5, self._publish_state)

            state_str = "PLAYING" if self._playing else "READY"
            self.get_logger().info(
                f"BVH replay: {bvh_path.name}, "
                f"{self._mapped_motion.frame_count} frames @ {rate_hz}Hz, "
                f"speed={playback_speed}x, loop={loop}, state={state_str}"
            )
            if not self._playing:
                self.get_logger().info(
                    "Waiting for start command. Robot is aligning to initial pose. "
                    "Call /teleop/start_playback or press Start Playback in GUI."
                )

        def _start_playback_cb(self, request, response):
            """Handle /teleop/start_playback service call."""
            if self._playing:
                response.success = True
                response.message = "Already playing"
                return response

            # Resume all adapters
            for adapter in self._trackers.values():
                adapter.resume()
            for adapter in self._hands.values():
                adapter.resume()

            self._playing = True
            self.get_logger().info("Playback started!")
            self._publish_state()

            response.success = True
            response.message = "Playback started"
            return response

        def _publish_state(self):
            """Publish current playback state as JSON."""
            state = "PLAYING" if self._playing else "READY"
            # Get progress from first tracker adapter
            progress = 0.0
            for adapter in self._trackers.values():
                progress = adapter.get_progress()
                break
            msg = String()
            msg.data = json.dumps({
                "state": state,
                "progress": progress,
            })
            self._playback_state_pub.publish(msg)

        def _offset_cb(self, msg) -> None:
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

        def _publish_all(self):
            now = self.get_clock().now()

            # Publish tracker poses
            for role, pub in self._tracker_pubs.items():
                adapter = self._trackers[role]
                pose = adapter.get_pose()
                if pose.valid:
                    # Apply calibration offset
                    position = pose.position.copy()
                    if role in self._calibration_offsets:
                        position = position + self._calibration_offsets[role]
                    msg = PoseStamped()
                    msg.header.stamp = now.to_msg()
                    msg.header.frame_id = "world"
                    msg.pose.position.x = float(position[0])
                    msg.pose.position.y = float(position[1])
                    msg.pose.position.z = float(position[2])
                    msg.pose.orientation.x = float(pose.orientation[0])
                    msg.pose.orientation.y = float(pose.orientation[1])
                    msg.pose.orientation.z = float(pose.orientation[2])
                    msg.pose.orientation.w = float(pose.orientation[3])
                    pub.publish(msg)

            # Publish HMD orientation
            head_adapter = self._trackers[TrackerRole.HEAD]
            head_pose = head_adapter.get_pose()
            if head_pose.valid:
                msg = QuaternionStamped()
                msg.header.stamp = now.to_msg()
                msg.header.frame_id = "hmd"
                msg.quaternion.x = float(head_pose.orientation[0])
                msg.quaternion.y = float(head_pose.orientation[1])
                msg.quaternion.z = float(head_pose.orientation[2])
                msg.quaternion.w = float(head_pose.orientation[3])
                self._hmd_pub.publish(msg)

            # Publish hand joint states
            for side, pub in self._hand_pubs.items():
                adapter = self._hands[side]
                state = adapter.get_joint_state()
                if state.valid:
                    msg = JointState()
                    msg.header.stamp = now.to_msg()
                    msg.position = state.joint_angles.tolist()
                    pub.publish(msg)

    rclpy.init()
    node = BVHReplayPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
