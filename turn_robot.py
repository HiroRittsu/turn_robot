import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rione_msgs.msg import Command
from numpy import pi
import numpy as np
from ekf import EKF
from std_msgs.msg import String


class TurnRobot(Node):
    def __init__(self):
        super().__init__("turn_robot")

        self._recieve_command = self.create_subscription(Command, "/turn_robot/command", self.recieve_command, 10)
        self._recieve_odom = self.create_subscription(Odometry, "/turtlebot2/odometry", self.recieve_odom, 10)
        self._status = self.create_publisher(String, "/turn_robot/status", 10)
        self._degree_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.last_angular_z = 0
        self.degree = 0.0
        self.target_degree = 0
        self.vel = None

        self._filter = EKF(0, 0, 0, 0.02)

    @staticmethod
    def to_angle(rad: float):
        return rad * 180 / np.pi

    def to_quaternion_ang(self, w: float, z: float):
        return abs((1 if z > 0 else 360) - self.to_angle(math.acos(w) * 2))

    def get_delta_angle(self, now_angle: float):
        delta_angle = now_angle - self.last_angular_z
        self.last_angular_z = now_angle
        if abs(delta_angle) > 180:
            delta_angle = (360 - abs(delta_angle)) * np.sign(delta_angle)
        return delta_angle

    def recieve_odom(self, msg: Odometry):
        now_angle = self.to_quaternion_ang(msg.pose.pose.orientation.w, msg.pose.pose.orientation.z)
        self.degree += self.get_delta_angle(now_angle)

        self.vel = Twist()
        self.vel.angular.z = 0.0

        print(self.degree)

        if 0 < self.target_degree:
            if self.degree < self.target_degree:
                self.vel.angular.z = pi / 6
            else:
                self._status.publish(String(data="FINISH"))
                self.target_degree = 0

        elif self.target_degree < 0:
            if self.degree > self.target_degree:
                self.vel.angular.z = -pi / 6
            else:
                self._status.publish(String(data="FINISH"))
                self.target_degree = 0

        self._degree_publisher.publish(self.vel)

    def recieve_command(self, msg):
        if msg.command == "START":
            self.target_degree = int(msg.content)
            self.degree = 0.0


def main():
    rclpy.init()
    _node = TurnRobot()
    rclpy.spin(_node)


if __name__ == "__main__":
    main()
