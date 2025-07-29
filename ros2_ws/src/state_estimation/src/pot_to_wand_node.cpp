#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>     // pot_data source (Int64)
#include <std_msgs/msg/float64.hpp>   // wand_angle_deg output

class PotToWandNode : public rclcpp::Node
{
public:
  PotToWandNode()
  : Node("pot_to_wand_node"),
    slope_(declare_parameter<double>("slope",  -0.035)),  // deg / count
    offset_(declare_parameter<double>("offset", 132.0))   // deg
  {
    auto qos = rclcpp::SensorDataQoS();   // best-effort, no history

    sub_ = create_subscription<std_msgs::msg::Int64>(
        "/pot_data", qos,
        [this](const std_msgs::msg::Int64::SharedPtr msg)
        {
          double raw = static_cast<double>(msg->data);

          /* Clamp to 12-bit ADC range */
          if (raw < 0.0)    raw = 0.0;
          if (raw > 4095.0) raw = 4095.0;

          std_msgs::msg::Float64 angle_msg;
          angle_msg.data = slope_ * raw + offset_;   // deg
          pub_->publish(angle_msg);
        });

    pub_ = create_publisher<std_msgs::msg::Float64>(
        "/wand_angle_deg", 10);

    RCLCPP_INFO(get_logger(),
      "pot_to_wand_node running  (slope %.4f, offset %.1f)", slope_, offset_);
  }

private:
  const double slope_;
  const double offset_;

  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  pub_;
};

/* ---- main ---- */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PotToWandNode>());
  rclcpp::shutdown();
  return 0;
}