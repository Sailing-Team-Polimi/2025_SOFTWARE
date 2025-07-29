#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>

class GpsAltitudeNode : public rclcpp::Node
{
public:
  GpsAltitudeNode() : Node("gps_altitude_node"), origin_set_(false)
  {
    // — subscriber —
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps_data", 10,
      std::bind(&GpsAltitudeNode::gps_callback, this, std::placeholders::_1));

    // — publisher —
    alt_pub_ = create_publisher<nav_msgs::msg::Odometry>("/gps/altitude", 10);
  }

private:
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
      return;                                          // niente fix → esci

    double alt = msg->altitude;                        // metri above ellipsoid

    if (!origin_set_) {                               // prima volta → fissa origine
      alt0_ = alt;
      origin_set_ = true;
      RCLCPP_INFO(get_logger(), "Altitude datum set to %.2f m", alt0_);
    }

    nav_msgs::msg::Odometry out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id  = "odom";
    out.child_frame_id   = "base_link";

    out.pose.pose.position.x = 0.0;                    // niente XY
    out.pose.pose.position.y = 0.0;
    out.pose.pose.position.z = alt - alt0_;            // quota relativa [m]

    out.pose.pose.orientation.w = 1.0;                 // senza orientamento

    // Covarianza: 0.1 m² su Z, 0 su tutto il resto
    for (double &c : out.pose.covariance) c = 0.0;
    out.pose.covariance[14] = 0.1;                     // indice 14 → var(z)

    alt_pub_->publish(out);
  }

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr        alt_pub_;

  // Stato interno
  bool   origin_set_;
  double alt0_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsAltitudeNode>());
  rclcpp::shutdown();
  return 0;
}