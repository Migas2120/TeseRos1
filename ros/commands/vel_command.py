import rospy
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from ros.commands.base_command import BaseCommand
from geometry_msgs.msg import Twist

class VelCommand(BaseCommand):
    def __init__(self, logger=None):
        super().__init__(logger)
        self.pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)

    def execute(self, ros, data: dict):
        try:
            twist = Twist()

            # Fill in linear velocity (x, y, z)
            twist.linear.x = float(data.get("x", 0.0))
            twist.linear.y = float(data.get("y", 0.0))
            twist.linear.z = float(data.get("z", 0.0))

            # Fill in angular z (yaw rate)
            twist.angular.z = float(data.get("yaw", 0.0))

            self.pub.publish(twist)
            self.logger.info(f"[VEL] Published velocity: {twist.linear.x}, {twist.linear.y}, {twist.linear.z} | yaw: {twist.angular.z}")

        except Exception as e:
            self.logger.error(f"[VEL] Exception: {e}")
