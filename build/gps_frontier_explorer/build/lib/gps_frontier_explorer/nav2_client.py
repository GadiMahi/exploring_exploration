from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math
import rclpy

def yaw_to_quat(yaw: float):
    half = yaw/2.0
    return [0.0, 0.0, math.sin(half), math.cos(half)]

class Nav2Client:

    def __init__(self):
        self.navigator = BasicNavigator()

        #wait until Nav2 is active (bringup must be running )
        self.navigator.waitUntilNav2Active()

    def go_to_xy(self, x: float, y: float, yaw: float = 0.0, frame_id: str = "map"):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quat(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.navigator.goToPose(pose)

    def is_task_complete(self) -> bool:
        return self.navigator.isTaskComplete()

    def result(self):
        return self.navigator.getResult()
    
    def cancel(self):
        self.navigator.cancelTask()

    def spin_once (self, timeout_sec: float = 0.1):
        rclpy.spin_once(self.navigator)

        