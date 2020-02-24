import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from rione_msgs.msg import Command

import numpy
from numpy import sin, cos
from numpy import deg2rad
from numpy import pi

from ekf import EKF

class turn_robot(Node):
    def __init__(self):
        super().__init__("turn_robot")

        self._recieve_command = self.create_subscription(Command, "/turn_robot", self.recieve_command, 10)
        self._recieve_odom    = self.create_subscription(Odometry, "/turtlebot2/odometry", self.recieve_odom, 10)

        self._degree_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.degree = 0.0
        self.target_degree = 0

        self._filter = EKF(0, 0, 0, 0.02)

    def recieve_odom(self, msg):

        self.degree += msg.twist.twist.angular.z

        self.vel = Twist()
        self.vel.angular.z = 0.0

        est_x, est_y, est_theta = self._filter.kalman_filter(0, 0, self.degree)
        print(est_theta)

        if 0 < self.target_degree:
            if self.degree < self.target_degree:
                self.vel.angular.z = pi/6

        elif self.target_degree < 0:
            if self.degree > self.target_degree:
                self.vel.angular.z = -pi/6
        
        self._degree_publisher.publish(self.vel)


    def recieve_command(self, msg):
        if msg.command == "START":
            self.target_degree = deg2rad(float(msg.content))
            self.degree = 0.0


    def euler_to_quaternion(self, yaw, pitch, roll):
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)

        quaternion = Quaternion()
        quaternion.x = qx
        quaternion.y = qy
        quaternion.z = qz
        quaternion.w = qw

        return quaternion


def main():
    rclpy.init()

    _node = turn_robot()

    rclpy.spin(_node)

if __name__ == "__main__":
    main()
