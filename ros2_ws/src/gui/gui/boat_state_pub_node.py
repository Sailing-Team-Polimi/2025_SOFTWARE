import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

import math as m

class BoatStatePub(Node):

    def __init__(self):
        super().__init__('boat_state_pub')

        self.states = JointState()
        self.states.name = ['rudder_joint','wand_joint','mast_joint']
        
        self.rudder_pose = float(0)
        self.wand_pose = float(0)
        self.mast_pose = float(0)

        self.wand_sub_ = self.create_subscription( Float64, 'wand_angle', self.wand_callback, 10)

        self.state_pub_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def wand_callback(self, msg):
        self.wand_pose = msg.data*m.pi/180
    
    def timer_callback(self):
        self.states.header.stamp = self.get_clock().now().to_msg()
        self.states.position = [float(self.rudder_pose), float(self.wand_pose), float(self.mast_pose)]
        
        self.state_pub_.publish(self.states)



def main(args=None):
    rclpy.init(args=args)

    state_pub_node = BoatStatePub()

    rclpy.spin(state_pub_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state_pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()