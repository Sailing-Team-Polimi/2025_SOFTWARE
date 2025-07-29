import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from tf_transformations import euler_from_quaternion
from sail_msgs.msg import PoliSailGuiMsg


class SendToMonitor(Node):
    def __init__(self):
        super().__init__('send_to_monitor')

        # Inizializzazione dei dati
        self.height = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.mod_vel = 0.0
        self.angolo_scarroccio = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0

        # Sottoscrizioni
        self.create_subscription(Odometry, '/odometry/local', self.odometry_local_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odometry_filtered_callback, 10)
        self.create_subscription(NavSatFix, '/gps_data', self.odometry_gps_callback, 10)

        # Publisher verso la WebApp
        self.publisher = self.create_publisher(PoliSailGuiMsg, '/sail_gui_data', 10)

        # Timer di pubblicazione (10Hz)
        self.create_timer(0.1, self.publish_data)

    def odometry_local_callback(self, msg: Odometry):
        self.height = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, _ = euler_from_quaternion(quaternion)

        self.roll = math.degrees(roll)
        self.pitch = math.degrees(pitch)

    def odometry_filtered_callback(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quaternion)

        # Yaw in NED
        heading_deg = (90 - math.degrees(yaw)) % 360
        self.yaw = heading_deg

        self.mod_vel = math.sqrt(vx ** 2 + vy ** 2)

        # Angolo di scarroccio
        course_degrees = math.degrees(math.atan2(vy, vx)) % 360
        
        self.angolo_scarroccio = (course_degrees - heading_deg + 540) % 360 - 180  # Normalizza tra -180 e 180 gradi
    
    def odometry_gps_callback(self, msg: NavSatFix):
        
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.alt = msg.altitude
        
        
    def publish_data(self):
        # Calcolo posizione globale simulata
        msg = PoliSailGuiMsg()
        msg.stamp = self.get_clock().now().to_msg()

        msg.height = Float32(data=self.height)
        msg.pitch = Float32(data=self.pitch)
        msg.roll = Float32(data=self.roll)
        msg.yaw = Float32(data=self.yaw)
        msg.mod_vel = Float32(data=self.mod_vel)
        msg.angolo_scarroccio = Float32(data=self.angolo_scarroccio)
        msg.lat = Float32(data=self.lat)
        msg.lon = Float32(data=self.lon)
        msg.alt = Float32(data=self.alt)

        self.publisher.publish(msg)

        # self.get_logger().info(
        #     f"[GUI → /sail_gui_data] "
        #     f"height={self.height:.2f} pitch={self.pitch:.2f} roll={self.roll:.2f} "
        #     f"yaw={self.yaw:.2f} mod_vel={self.mod_vel:.2f} scarroccio={self.angolo_scarroccio:.2f} "
        #     f"lat={self.lat_global:.6f} lon={self.lon_global:.6f}"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = SendToMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
