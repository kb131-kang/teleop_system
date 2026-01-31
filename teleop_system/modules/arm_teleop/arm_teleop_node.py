"""ROS2 Lifecycle Node for arm teleoperation.

Subscribes to master tracker poses, runs IK via ArmController,
and publishes joint commands to the slave robot.

This node can operate independently of other modules,
communicating only through ROS2 topics.
"""

from pathlib import Path

from teleop_system.utils.logger import get_logger

logger = get_logger("arm_teleop_node")

try:
    import rclpy
    from rclpy.lifecycle import Node as LifecycleNode
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState

    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False
    logger.warning("ROS2 not available — ArmTeleopNode cannot be used")


if _ROS2_AVAILABLE:
    from teleop_system.utils.ros2_helpers import (
        QoSPreset,
        TopicNames,
        get_qos_profile,
        create_timer_rate,
    )
    from teleop_system.modules.arm_teleop.ros2_adapters import (
        ROS2TrackerAdapter,
        ROS2ArmCommandPublisher,
    )
    from teleop_system.modules.arm_teleop.arm_controller import ArmController
    from teleop_system.interfaces.master_device import TrackerRole
    from teleop_system.interfaces.slave_robot import ArmSide
    from teleop_system.solvers.proportional_mapper import create_ik_solver

    class ArmTeleopNode(LifecycleNode):
        """ROS2 Lifecycle Node for arm teleoperation.

        Subscribes to:
            - /master/tracker/{left,right,waist} (PoseStamped)
        Publishes:
            - /slave/arm/{left,right}/joint_cmd (JointState)
            - /slave/torso/joint_cmd (JointState)

        Internally creates ROS2 adapter classes to bridge ROS2 messages
        with ArmController's IMasterTracker/ISlaveArm interfaces.
        """

        def __init__(self, node_name: str = "arm_teleop_node"):
            super().__init__(node_name)

            self._cb_group = MutuallyExclusiveCallbackGroup()

            # ── ROS2 Subscribers ──
            sensor_qos = get_qos_profile(QoSPreset.SENSOR_DATA)

            self.create_subscription(
                PoseStamped,
                TopicNames.TRACKER_RIGHT_HAND,
                lambda msg: self._tracker_callback("right", msg),
                sensor_qos,
                callback_group=self._cb_group,
            )
            self.create_subscription(
                PoseStamped,
                TopicNames.TRACKER_LEFT_HAND,
                lambda msg: self._tracker_callback("left", msg),
                sensor_qos,
                callback_group=self._cb_group,
            )
            self.create_subscription(
                PoseStamped,
                TopicNames.TRACKER_WAIST,
                lambda msg: self._tracker_callback("waist", msg),
                sensor_qos,
                callback_group=self._cb_group,
            )

            # ── ROS2 Publishers ──
            cmd_qos = get_qos_profile(QoSPreset.COMMAND)

            self._pub_right = self.create_publisher(
                JointState, TopicNames.ARM_RIGHT_CMD, cmd_qos
            )
            self._pub_left = self.create_publisher(
                JointState, TopicNames.ARM_LEFT_CMD, cmd_qos
            )
            self._pub_torso = self.create_publisher(
                JointState, TopicNames.TORSO_CMD, cmd_qos
            )

            # ── ROS2 Tracker Adapters (IMasterTracker wrappers) ──
            self._tracker_right = ROS2TrackerAdapter(TrackerRole.RIGHT_HAND)
            self._tracker_left = ROS2TrackerAdapter(TrackerRole.LEFT_HAND)
            self._tracker_waist = ROS2TrackerAdapter(TrackerRole.WAIST)

            self._tracker_map = {
                "right": self._tracker_right,
                "left": self._tracker_left,
                "waist": self._tracker_waist,
            }

            # ── ROS2 Arm Command Publishers (ISlaveArm wrappers) ──
            self._arm_pub_right = ROS2ArmCommandPublisher(
                side=ArmSide.RIGHT,
                publisher=self._pub_right,
                clock=self.get_clock(),
                n_joints=7,
            )
            self._arm_pub_left = ROS2ArmCommandPublisher(
                side=ArmSide.LEFT,
                publisher=self._pub_left,
                clock=self.get_clock(),
                n_joints=7,
            )
            self._arm_pub_torso = ROS2ArmCommandPublisher(
                side=ArmSide.TORSO,
                publisher=self._pub_torso,
                clock=self.get_clock(),
                n_joints=6,
            )

            # ── IK Solver (PinkIKSolver if available, else proportional) ──
            ik_solver = create_ik_solver(prefer_pink=True)
            joint_index_map = self._init_ik_solver(ik_solver)

            # ── ArmController (pure control logic) ──
            self._arm_controller = ArmController(
                ik_solver=ik_solver,
                trackers={
                    TrackerRole.RIGHT_HAND: self._tracker_right,
                    TrackerRole.LEFT_HAND: self._tracker_left,
                    TrackerRole.WAIST: self._tracker_waist,
                },
                arms={
                    ArmSide.RIGHT: self._arm_pub_right,
                    ArmSide.LEFT: self._arm_pub_left,
                    ArmSide.TORSO: self._arm_pub_torso,
                },
                dt=0.01,
                joint_index_map=joint_index_map,
            )
            self._arm_controller.enable()

            # ── Control Loop Timer (100 Hz) ──
            self._control_timer = self.create_timer(
                create_timer_rate(100.0),
                self._control_loop,
                callback_group=self._cb_group,
            )

            self.get_logger().info("ArmTeleopNode created with ArmController integration")

        def _init_ik_solver(self, solver) -> dict:
            """Initialize the IK solver and return the joint_index_map.

            Returns:
                Dict mapping ArmSide -> list of indices into the IK solver's
                q vector for that chain's joints.
            """
            from teleop_system.solvers.proportional_mapper import SimpleProportionalMapper

            if isinstance(solver, SimpleProportionalMapper):
                solver.initialize("", nq=20, chain_joint_indices={
                    "torso": list(range(0, 6)),
                    "right_arm": list(range(6, 13)),
                    "left_arm": list(range(13, 20)),
                })
                self.get_logger().info("Using SimpleProportionalMapper (fallback)")
                return {
                    ArmSide.TORSO: list(range(0, 6)),
                    ArmSide.RIGHT: list(range(6, 13)),
                    ArmSide.LEFT: list(range(13, 20)),
                }
            else:
                # PinkIKSolver: needs patched URDF and actual EE frame names
                project_root = Path(__file__).resolve().parent.parent.parent.parent
                urdf_path = str(project_root / "models" / "rby1" / "urdf" / "model_pinocchio.urdf")
                success = solver.initialize(
                    urdf_path,
                    ee_frames={
                        "right_arm": "ee_right",
                        "left_arm": "ee_left",
                        "torso": "link_torso_5",
                    },
                )
                if success:
                    self.get_logger().info("Using PinkIKSolver")
                    # Pinocchio q-vector indices for each chain
                    return {
                        ArmSide.TORSO: [4, 5, 6, 7, 8, 9],       # torso_0..5
                        ArmSide.RIGHT: [21, 22, 23, 24, 25, 26, 27],  # right_arm_0..6
                        ArmSide.LEFT: [12, 13, 14, 15, 16, 17, 18],   # left_arm_0..6
                    }
                else:
                    self.get_logger().warn("PinkIKSolver init failed, arm teleop may not work")
                    return {
                        ArmSide.TORSO: list(range(0, 6)),
                        ArmSide.RIGHT: list(range(6, 13)),
                        ArmSide.LEFT: list(range(13, 20)),
                    }

        def _tracker_callback(self, tracker_id: str, msg: PoseStamped) -> None:
            """Feed incoming PoseStamped into the ROS2TrackerAdapter."""
            adapter = self._tracker_map.get(tracker_id)
            if adapter is not None:
                adapter.update_pose(msg)

        def _control_loop(self) -> None:
            """Timer callback: run one ArmController update cycle.

            ArmController reads from ROS2TrackerAdapter (get_pose),
            solves IK, and dispatches via ROS2ArmCommandPublisher (send_joint_command).
            """
            result = self._arm_controller.update()
            if result.success:
                self.get_logger().debug(
                    f"IK solved: pos_err={result.error_position:.4f} "
                    f"ori_err={result.error_orientation:.4f}",
                    throttle_duration_sec=2.0,
                )

    def main(args=None):
        """Entry point for the arm teleop node."""
        rclpy.init(args=args)
        node = ArmTeleopNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

else:
    # Stub for environments without ROS2
    class ArmTeleopNode:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("ROS2 is not available. Install ros-jazzy-desktop.")

    def main(args=None):
        raise RuntimeError("ROS2 is not available. Install ros-jazzy-desktop.")


if __name__ == "__main__":
    main()
