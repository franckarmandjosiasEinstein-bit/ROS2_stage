"""arm_node -- scripted 5-DOF pick-and-place for the YouBot arm in Gazebo.

On a /do_pick trigger it plays a smooth keyframe sequence (stow -> reach ->
grasp -> lift -> place -> release -> stow) by ramping joint targets, then
publishes /pick_done. Each joint target is sent to its gz JointPositionController
(a Float64 per joint, bridged to gz Double); the same angles are published on
/joint_states so robot_state_publisher animates the arm in RViz too.

Subscribes:  /do_pick   (std_msgs/Empty)   start one pick cycle
Publishes:   /arm_joint_1_cmd .. /gripper_right_cmd  (std_msgs/Float64) -> gz
             /joint_states  (sensor_msgs/JointState) -> robot_state_publisher
             /pick_done     (std_msgs/Empty)   emitted when the cycle finishes
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, Float64
from sensor_msgs.msg import JointState

ARM_JOINTS = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
GRIP_JOINTS = ["gripper_left", "gripper_right"]

OPEN = 0.030    # m: finger opening
CLOSED = 0.006  # m

# Keyframes: [j1, j2, j3, j4, j5, grip]. The arm is column-mounted and faces the
# +Y plant row; j2/j3/j4 tip it out of vertical toward the row at ~0.9 m; j1
# swings it back over the robot to drop. First-pass angles, tuned from Gazebo.
STOW = [0.0, 0.3, -2.2, -1.0, 0.0, OPEN]    # folded compact over the column
READY = [0.0, 1.0, -1.2, -0.4, 0.0, OPEN]   # swung out toward the row
REACH = [0.0, 1.3, -0.9, -0.3, 0.0, OPEN]   # extended into the row at fruit height
GRASP = [0.0, 1.3, -0.9, -0.3, 0.0, CLOSED]  # close on the fruit
LIFT = [0.0, 0.9, -1.2, -0.4, 0.0, CLOSED]   # lift clear of the plant
PLACE = [1.6, 0.7, -1.5, -0.4, 0.0, CLOSED]  # swing back over the robot to a bin
RELEASE = [1.6, 0.7, -1.5, -0.4, 0.0, OPEN]  # drop

PICK_SEQUENCE = [READY, REACH, GRASP, LIFT, PLACE, RELEASE, STOW]

ARM_STEP = 0.03    # rad per tick (30 Hz -> ~1 rad in ~1.1 s)
GRIP_STEP = 0.0025  # m per tick


def _sign(x):
    return 1.0 if x >= 0.0 else -1.0


class ArmNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_node")
        self._q = list(STOW)          # current joint values
        self._queue = []              # list of keyframes still to reach
        self._picking = False

        self._cmd_pubs = {
            j: self.create_publisher(Float64, f"{j}_cmd", 10)
            for j in ARM_JOINTS + GRIP_JOINTS
        }
        self._js_pub = self.create_publisher(JointState, "joint_states", 10)
        self._done_pub = self.create_publisher(Empty, "pick_done", 5)
        self.create_subscription(Empty, "do_pick", self._on_do_pick, 5)

        self.create_timer(1.0 / 30.0, self._tick)
        self.get_logger().info(
            "arm_node up: /do_pick -> pick sequence; cmds + /joint_states + /pick_done")

    def _on_do_pick(self, _msg: Empty) -> None:
        if self._picking:
            return  # already busy
        self._picking = True
        self._queue = [list(k) for k in PICK_SEQUENCE]
        self.get_logger().info("Pick sequence started.")

    def _tick(self) -> None:
        if self._queue:
            tgt = self._queue[0]
            reached = True
            for i in range(6):
                step = GRIP_STEP if i == 5 else ARM_STEP
                d = tgt[i] - self._q[i]
                if abs(d) <= step:
                    self._q[i] = tgt[i]
                else:
                    self._q[i] += step * _sign(d)
                    reached = False
            if reached:
                self._queue.pop(0)
                if not self._queue and self._picking:
                    self._picking = False
                    self._done_pub.publish(Empty())
                    self.get_logger().info("Pick sequence done.")
        self._publish()

    def _publish(self) -> None:
        # arm joints
        for i, j in enumerate(ARM_JOINTS):
            self._cmd_pubs[j].publish(Float64(data=float(self._q[i])))
        # both fingers share the opening value
        for j in GRIP_JOINTS:
            self._cmd_pubs[j].publish(Float64(data=float(self._q[5])))

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ARM_JOINTS + GRIP_JOINTS
        js.position = [float(v) for v in self._q[:5]] + [float(self._q[5]), float(self._q[5])]
        self._js_pub.publish(js)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
