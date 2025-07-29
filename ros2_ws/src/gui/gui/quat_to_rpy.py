#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Vector3
import math

def quaternion_to_euler(x, y, z, w):
    # Calculate roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    # Calculate pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)  # Clamp t2 to ensure it is in the range [-1, 1]
    pitch = math.asin(t2)
    
    # Calculate yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return roll, pitch, yaw

class QuaternionToRPY(Node):
    def __init__(self):
        super().__init__('quaternion_to_rpy_node')
        # Subscriber: read from /quaternion topic
        self.subscription = self.create_subscription(
            Quaternion,
            '/quaternion',
            self.quaternion_callback,
            10)
        # Publisher: write to /est/rpy topic
        self.publisher = self.create_publisher(Vector3, '/est/rpy', 10)

    def quaternion_callback(self, msg):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = quaternion_to_euler(msg.x, msg.y, msg.z, msg.w)
        
        # Create a Vector3 message with the computed angles
        rpy_msg = Vector3()
        rpy_msg.x = roll
        rpy_msg.y = pitch
        rpy_msg.z = yaw
        
        # Publish the RPY message
        self.publisher.publish(rpy_msg)
        self.get_logger().info(
            f'Published RPY: roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = QuaternionToRPY()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
