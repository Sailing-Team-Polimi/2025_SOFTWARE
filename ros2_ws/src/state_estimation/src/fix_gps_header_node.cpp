#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using std::placeholders::_1;

class FixGPSHeader : public rclcpp::Node
{
public:
  FixGPSHeader()
  : Node("fix_gps_header")
  {
    this->declare_parameter("frame_id", "base_link");
    this->get_parameter("frame_id", frame_id_);
    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps_data", 10, std::bind(&FixGPSHeader::topic_callback, this, _1));

    publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/data_fixed", 10);


    RCLCPP_INFO(this->get_logger(), "Nodo fix_gps_header avviato. Frame ID impostato su: %s", frame_id_.c_str());
  }

private:
  void topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    auto fixed_msg = *msg;
    fixed_msg.header.frame_id = frame_id_;
    fixed_msg.header.stamp = this->get_clock()->now();
    fixed_msg.position_covariance[0]=0.5;
    fixed_msg.position_covariance[4]=0.5;
    fixed_msg.position_covariance[8]=1.0;
    fixed_msg.position_covariance_type= sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    publisher_->publish(fixed_msg);
    RCLCPP_DEBUG(this->get_logger(), "Pubblicato messaggio GPS corretto con frame_id: %s", frame_id_.c_str());
  }
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
  std::string frame_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixGPSHeader>());
  rclcpp::shutdown();
  return 0;
} 
