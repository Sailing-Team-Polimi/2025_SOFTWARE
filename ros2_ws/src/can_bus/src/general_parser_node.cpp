#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "sail_msgs/msg/serial_msg.hpp"
#include <vector>
#include <cstdint>

class GeneralParser : public rclcpp::Node
{
public:
  GeneralParser()
  : Node("general_parser")
  {
    filter_id_    = this->declare_parameter<int>("id",     0);
    pack_length_  = this->declare_parameter<int>("length", 2);
    scale_        = this->declare_parameter<int>("scale", 100);

    subscription_ = this->create_subscription<sail_msgs::msg::SerialMsg>(
      "/raw_data", 10,
      std::bind(&GeneralParser::topic_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::Float64>("/parsed_data", 10);
  }

private:
  void topic_callback(const sail_msgs::msg::SerialMsg::SharedPtr msg)
  {
    // 1) Filter by ID
    RCLCPP_DEBUG(this->get_logger(), "gotten id %u (expected %u)",msg->id, static_cast<uint8_t>(filter_id_));
    if (msg->id != static_cast<uint8_t>(filter_id_)) {
      return;
    }

    // 2) Check payload length
    if (msg->payload.data.size() != static_cast<size_t>(pack_length_)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Unexpected payload size %zu (expected %d)",
        msg->payload.data.size(), pack_length_);
      return;
    }

    // 3) Convert bytes → int64
    int64_t value = byteArray2Int(msg->payload.data);

    // 4) Publish
    auto out = std_msgs::msg::Float64();
    out.data = float(value);
    publisher_->publish(out);
  }

  /// Interpret a big-endian byte array as a signed 64-bit integer.
  int64_t byteArray2Int(const std::vector<uint8_t> & bytes)
  {
    int64_t result = 0;
    for (auto b : bytes) {
      result = (result << 8) | static_cast<int64_t>(b);
    }
    return result;
  }

  int filter_id_;
  int pack_length_;
  int scale_;
  rclcpp::Subscription<sail_msgs::msg::SerialMsg>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr       publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GeneralParser>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
