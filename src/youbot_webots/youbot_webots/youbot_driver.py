"""webots_ros2 driver plugin for the YouBot mecanum base.

webots_ros2 auto-publishes most sensors (lidar -> /scan, camera -> image)
from the <device> tags in the URDF. What it cannot do generically is the
mecanum /cmd_vel -> 4 wheel-speeds mapping, and turning GPS + Compass into a
single /odom pose. This plugin does exactly those two jobs.

Lifecycle (called by webots_ros2_driver):
    init(webots_node, properties)  once, to grab devices and make pub/subs
    step()                         every Webots basic time step

Wire it in from the URDF:
    <webots><plugin type="youbot_webots.youbot_driver.YoubotDriver"/></webots>
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from youbot_control.lib.mecanum import body_to_wheel_speeds

WHEEL_NAMES = ("wheel1", "wheel2", "wheel3", "wheel4")


class YoubotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        timestep = int(self.__robot.getBasicTimeStep())

        # Base wheels: velocity control (infinite position target).
        self.__wheels = [self.__robot.getDevice(n) for n in WHEEL_NAMES]
        for w in self.__wheels:
            w.setPosition(float("inf"))
            w.setVelocity(0.0)

        # Pose sensors.
        self.__gps = self.__robot.getDevice("gps")
        self.__compass = self.__robot.getDevice("compass")
        if self.__gps:
            self.__gps.enable(timestep)
        if self.__compass:
            self.__compass.enable(timestep)

        # ROS 2 interface. The webots_controller process does not init rclpy
        # for us, so do it here (guarded in case a future version does).
        if not rclpy.ok():
            rclpy.init(args=None)
        self.__node = rclpy.create_node("youbot_driver")
        self.__target = Twist()
        self.__node.create_subscription(Twist, "cmd_vel", self.__on_cmd_vel, 1)
        self.__odom_pub = self.__node.create_publisher(Odometry, "odom", 1)
        self.__node.get_logger().info("YoubotDriver ready: /cmd_vel -> wheels, GPS+Compass -> /odom")

    def __on_cmd_vel(self, msg: Twist) -> None:
        self.__target = msg

    def step(self) -> None:
        rclpy.spin_once(self.__node, timeout_sec=0)

        # 1) Drive the base from the latest /cmd_vel.
        speeds = body_to_wheel_speeds(
            self.__target.linear.x, self.__target.linear.y, self.__target.angular.z)
        for w, s in zip(self.__wheels, speeds):
            w.setVelocity(s)

        # 2) Publish /odom from GPS (position) + Compass (heading).
        if self.__gps and self.__compass:
            p = self.__gps.getValues()
            n = self.__compass.getValues()
            if not any(math.isnan(v) for v in p[:2]):
                yaw = math.atan2(n[0], n[1])
                self.__odom_pub.publish(self.__make_odom(p[0], p[1], yaw))

    def __make_odom(self, x, y, yaw) -> Odometry:
        msg = Odometry()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return msg
