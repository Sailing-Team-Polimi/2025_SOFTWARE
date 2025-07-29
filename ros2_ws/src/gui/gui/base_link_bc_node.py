import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Vector3, Pose, Quaternion
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R

import math as m
import numpy as np

def quaternion_from_euler(roll, pitch, yaw):
   """
   Converts euler roll, pitch, yaw to quaternion (w in last place)
   quat = [x, y, z, w]
   Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
   """
   cy = m.cos(yaw * 0.5)
   sy = m.sin(yaw * 0.5)
   cp = m.cos(pitch * 0.5)
   sp = m.sin(pitch * 0.5)
   cr = m.cos(roll * 0.5)
   sr = m.sin(roll * 0.5)
   
   q = Quaternion()
   q.x = cy * cp * cr + sy * sp * sr
   q.y = cy * cp * sr - sy * sp * cr
   q.z = sy * cp * sr + cy * sp * cr
   q.w = sy * cp * cr - cy * sp * sr
   
   return q


class TfBroadcaster(Node):

    def __init__(self):
        super().__init__('tf_broadcaster')

        self.tf_br = TransformBroadcaster(self)

        self.xy_sub_ = self.create_subscription(PoseWithCovarianceStamped,'orientation', self.orientation_callback, 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.pose = Pose()


    
    def orientation_callback(self,msg):
        self.pose.orientation = msg.pose.pose.orientation
    
    def timer_callback(self):
        
        self.tf = TransformStamped()
        self.tf.header.frame_id = "odom"
        self.tf.child_frame_id = "base_link"
        
        self.tf.transform.translation.x = self.pose.position.x
        self.tf.transform.translation.y = self.pose.position.y
        self.tf.transform.translation.z = self.pose.position.z
        self.tf.transform.rotation = self.pose.orientation
        
        self.tf.header.stamp = self.get_clock().now().to_msg()
        self.tf_br.sendTransform(self.tf)


def main(args=None):
    rclpy.init(args=args)

    tf_broadcaster = TfBroadcaster()

    rclpy.spin(tf_broadcaster)

    tf_broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()