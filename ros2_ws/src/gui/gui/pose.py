import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, Vector3, Pose, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

import math as m
import numpy as np

class SetPose(Node):

    def __init__(self):
        super().__init__('set_pose_node')
        self.h_sub = self.create_subscription( Float32, 'h_est', self.height_callback, 10)
        self.v_sub = self.create_subscription( Twist, 'v_est', self.v_callback, 10)
        self.ypr_sub = self.create_subscription( Vector3, 'ypr_est', self.ypr_callback, 10)
        
        self.pose_pub = self.create_publisher(Pose, 'pose_est', 10)
        self.tf_br = TransformBroadcaster(self)

        self.moving_pose = Pose()
        self.ypr_pose = Pose()
        self.v_read = Twist()
        

        self.dt = 0.01  # seconds
        self.timer = self.create_timer(self.dt, self.publish_callback)

    def height_callback(self, msg):
        self.moving_pose.position.z = msg.data

    def v_callback(self,msg):
        self.v_read = msg
        self.v_read = msg

    def ypr_callback(self,msg):
        self.ypr_pose.orientation = self.quaternion_from_euler(msg.x,msg.y,msg.z)
    
    def quaternion_from_euler(self,roll, pitch, yaw):
        
        q = Quaternion()
        
        cy = m.cos(yaw * 0.5)
        sy = m.sin(yaw * 0.5)
        cp = m.cos(pitch * 0.5)
        sp = m.sin(pitch * 0.5)
        cr = m.cos(roll * 0.5)
        sr = m.sin(roll * 0.5)

        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr

        return q
    
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def publish_callback(self):
        self.update_pose()
        pose = self.moving_pose
        #pose.orientation = self.ypr_pose.orientation
        self.pose_pub.publish(pose)

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "boat_cm"
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation

        self.tf_br.sendTransform(t)

    def update_pose(self):

        roll, pitch, yaw = self.euler_from_quaternion(self.moving_pose.orientation)
        yaw += float(self.v_read.angular.z*self.dt)

        self.moving_pose.position.x += self.v_read.linear.x*m.cos(yaw)*self.dt 
        self.moving_pose.position.y += self.v_read.linear.x*m.sin(yaw)*self.dt 

        self.moving_pose.orientation = self.quaternion_from_euler(roll,pitch,yaw)





def main(args=None):
    rclpy.init(args=args)

    set_pose_node = SetPose()

    rclpy.spin(set_pose_node)

    set_pose_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()