import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import gpsd

class GpsPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        
        # Declare parameters for frequency and covariance
        self.declare_parameter('freq', 1.0)  # Publishing frequency in Hz
        self.declare_parameter('covariance', [0.1, 0.0, 0.0,
                                                0.0, 0.1, 0.0,
                                                0.0, 0.0, 0.1])
        freq = self.get_parameter('freq').value
        self.covariance = self.get_parameter('covariance').value

        self.publisher_ = self.create_publisher(NavSatFix, 'gps_data', 10)

        # Initialize gpsd session
        gpsd.connect()

        # Create a timer with period set by the frequency parameter
        self.timer = self.create_timer(1.0 / freq, self.publish_gps_data)

    def publish_gps_data(self):
        # Get GPS data
        gps_data = gpsd.get_current()

        msg = NavSatFix()
        
        # Add header with current time stamp and frame_id 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Set GPS data
        msg.latitude = gps_data.lat
        msg.longitude = gps_data.lon
        msg.altitude = gps_data.alt

        # Set covariance from parameter
        msg.position_covariance = self.covariance
        msg.position_covariance_type = 2

        # Log and publish the message
        self.get_logger().info(f"Publishing GPS data: {msg.latitude}, {msg.longitude}, {msg.altitude}")
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GpsPublisher()
    
    try:
        rclpy.spin(gps_publisher)
    except KeyboardInterrupt:
        pass

    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
