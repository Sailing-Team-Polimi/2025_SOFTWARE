#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/float64.hpp"

class IntToFloatNode : public rclcpp::Node
{
public:
    IntToFloatNode() : Node("pot2wand")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Int64>(
            "pot_data", 10,
            std::bind(&IntToFloatNode::callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Float64>("wand_angle", 10);

        RCLCPP_INFO(this->get_logger(), "Node started: converting pot_data (Int64 [0–4096]) to wand_angle (Float64 [0.0–2pi])");
    }

private:
    void callback(const std_msgs::msg::Int64::SharedPtr msg)
    {
        // Clamp input to [0, 4096]
        int64_t input = std::max<int64_t>(0, std::min<int64_t>(4096, msg->data));

        // Map from 0–4096 to 0.0–2pi
        double output = (static_cast<double>(input) / 4096.0) * 2 * M_PI;

        std_msgs::msg::Float64 out_msg;
        out_msg.data = output;

        publisher_->publish(out_msg);
        RCLCPP_INFO(this->get_logger(), "Received pot_data: %ld -> Published wand_angle: %f", msg->data, output);
    }

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IntToFloatNode>());
    rclcpp::shutdown();
    return 0;
}
