import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64
import random

class MockPublisher(Node):
    def __init__(self):
        super().__init__('mock_publisher')

        # Create publishers
        self.pub_heel = self.create_publisher(Float32, '/heel', 10)
        self.pub_pitch = self.create_publisher(Float32, '/pitch', 10)
        self.pub_start = self.create_publisher(Float32, '/start_timer', 10)
        self.pub_sog = self.create_publisher(Float32, '/sog', 10)
        self.pub_vmg = self.create_publisher(Float32, '/vmg', 10)
        self.pub_twa = self.create_publisher(Float32, '/twa', 10)
        self.pub_twd = self.create_publisher(Float32, '/twd', 10)
        self.pub_tws = self.create_publisher(Float32, '/tws', 10)
        self.pub_lat = self.create_publisher(Float64, '/lat', 10)
        self.pub_lon = self.create_publisher(Float64, '/lon', 10)

        # Start periodic publishing
        self.create_timer(0.1, self.publish_mock_data)  # 10Hz

    def publish_mock_data(self):
        # Generate random values
        heel = Float32(data=random.uniform(-30, 30))
        pitch = Float32(data=random.uniform(-30, 30))
        start_timer = Float32(data=random.uniform(0, 60))
        sog = Float32(data=random.uniform(0, 10))
        vmg = Float32(data=random.uniform(0, 10))
        twa = Float32(data=random.uniform(0, 180))
        twd = Float32(data=random.uniform(0, 360))
        tws = Float32(data=random.uniform(0, 50))
        lat = Float64(data=random.uniform(45.478, 45.479))
        lon = Float64(data=random.uniform(9.229, 9.230))

        # Publish
        self.pub_heel.publish(heel)
        self.pub_pitch.publish(pitch)
        self.pub_start.publish(start_timer)
        self.pub_sog.publish(sog)
        self.pub_vmg.publish(vmg)
        self.pub_twa.publish(twa)
        self.pub_twd.publish(twd)
        self.pub_tws.publish(tws)
        self.pub_lat.publish(lat)
        self.pub_lon.publish(lon)

        # Log what was published
        # self.get_logger().info(
        #     f"[Mock] heel={heel.data:.2f}, pitch={pitch.data:.2f}, timer={start_timer.data:.1f}, "
        #     f"sog={sog.data:.2f}, vmg={vmg.data:.2f}, twa={twa.data:.1f}, twd={twd.data:.1f}, "
        #     f"tws={tws.data:.1f}, lat={lat.data:.6f}, lon={lon.data:.6f}"
        # )

def main(args=None):
    rclpy.init(args=args)
    node = MockPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
