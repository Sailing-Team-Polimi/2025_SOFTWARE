import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64
from geometry_msgs.msg import PoseWithCovariance, TransformStamped
from tf2_ros import TransformBroadcaster

import math as m
import numpy as np

class TfBroadcaster(Node):

    def __init__(self):
        super().__init__('tf_broadcaster')

        self.tf_br = TransformBroadcaster(self)
        self.tf = TransformStamped()
        self.tf.header.frame_id = "world"
        self.tf.child_frame_id = "base_link"

        self.xy_sub_ = self.create_subscription(PoseWithCovariance,'est_xy', self.xy_callback, 10)
        self.z_sub_ = self.create_subscription(Float64,'est_z', self.z_callback, 10)

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def xy_callback(self, msg):
        self.tf.transform.translation.x = msg.pose.position.x
        self.tf.transform.translation.y = msg.pose.position.y

    def z_callback(self, msg):
        self.tf.transform.translation.z = msg.data
    
    def timer_callback(self):
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