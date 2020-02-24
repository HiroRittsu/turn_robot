import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rione_msgs.msg import Command

import numpy as np


class Turn(Node):
    def __init__(self):
        super().__init__("Trun")
        self.sender = ""
        self.command = ""
        self.turn = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )
        self.resetPose = self.create_publisher(
            Bool,
            "/turtlebot2/commands/reset_pose",
            10
        )
        self.create_subscription(
            Odometry,
            "/turtlebot2/odometry",
            self.send_velocity,
            10
        )
        self.create_subscription(
            Command,
            "/turn_robot/command",
            self.receive_flag,
            10
        )
        self.status = self.create_publisher(
            String,
            "/turn_robot/status",
            10
        )
        self.velocity = Twist()
        self.velocity.angular.z = 0.0
        self.setVelocity = False
        self.did = False
        self.degree = None

    def send_velocity(self, msg):
        w = msg.pose.pose.orientation.w
        z = msg.pose.pose.orientation.z

        if z > 0:
            angle = -np.arccos(w) * 360 / np.pi
        else:
            angle = np.arccos(w) * 360 / np.pi

        try:
            # if reach target angular
            if 0 <= int(self.degree) < 180:
                # print("0 ~ 180 , target={0} now={1}".format(self.degree, angle), flush=True)

                if int(self.degree) - 1 < angle < int(self.degree) + 1:

                    self.velocity.angular.z = 0.0

                    # if not send finish flag to sender
                    if not self.did:
                        print("[*] STOP TURN {0} DEGREE".format(self.degree), flush=True)

                        # send finish flag to sender
                        if self.sender == "sound":
                            print("sound_system", "Command:finish,Content:None")

                        if self.sender == "cerebrum":
                            print("cerebrum", "Command:{0},Content:None".format(self.command))

                        self.did = True
                        self.setVelocity = False

            if -180 <= int(self.degree) < 0:
                # print("-180 ~ 0 , target={0} now={1}".format(self.degree, angle), flush=True)

                if int(self.degree) - 1 < angle < int(self.degree) + 1:

                    self.velocity.angular.z = 0.0

                    # if not send finish flag to sender
                    if not self.did:
                        print("[*] STOP TURN {0} DEGREE".format(self.degree), flush=True)

                        # send finish flag to sender
                        if self.sender == "sound":
                            print("sound_system", "Command:finish,Content:None")

                        if self.sender == "cerebrum":
                            print("cerebrum", "Command:{0},Content:None".format(self.command))

                        self.did = True
                        self.setVelocity = False

        except TypeError:
            pass

        if self.setVelocity:
            if -180 <= int(self.degree) < 0:
                self.velocity.angular.z = np.pi * 30.0 / 180.0
            else:
                self.velocity.angular.z = -np.pi * 30.0 / 180.0

        self.turn.publish(self.velocity)

    def receive_flag(self, msg):
        self.command = msg.command
        content = msg.content

        self.degree = int(content)

        print(self.degree, flush=True)

        self.degree = self.degree % 180

        if 0 < self.degree <= 180:
            pass
        elif -180 < self.degree < 0:
            pass
        else:
            pass

        self.sender = msg.sender

        if self.command == "START":
            reset_flag = Bool()
            reset_flag.data = True
            self.did = False
            self.resetPose.publish(reset_flag)
            self.status.publish(String(data="FINISH"))
            self.setVelocity = True
            print("[*] START TURN {0} DEGREE".format(self.degree), flush=True)


def main():
    rclpy.init()
    node = Turn()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
