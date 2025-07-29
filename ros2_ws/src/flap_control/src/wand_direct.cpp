#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>

#include "sail_msgs/msg/serial_msg.hpp"

class Wand2ServoNode : public rclcpp::Node
{
public:
    Wand2ServoNode() : Node("wand_direct")
    {
        uint8_t servo_id_ = 0;

        this->declare_parameter<int>("servo_id", 10);

        // Creazione del publisher e del subscriber
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("servo_angle", 10);

        this->get_parameter("servo_id", servo_id_);

        serial_publisher_ = this->create_publisher<sail_msgs::msg::SerialMsg>("toBrunilde", 10);

        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "wand_angle", 10, std::bind(&Wand2ServoNode::wand_callback, this, std::placeholders::_1));
    }

private:
    void wand_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Servo angle is the angle read by the wand
        double wand_angle = msg->data;

        double servo_angle = 0.0;

        float m = 1;
        float q = 45;

        // Wand angle is in the range [0.0, 90]
        servo_angle = m * wand_angle + q;

        // 3. Create the message to publish to the "servo" topic
        auto new_msg = std_msgs::msg::Float64();
        new_msg.data = servo_angle;

        // 4. Publish the message to the "servo" topic
        RCLCPP_INFO(this->get_logger(), "Received wand angle: '%f', sending servo angle: '%f'", msg->data, servo_angle);
        publisher_->publish(new_msg);

        auto toSerial = sail_msgs::msg::SerialMsg();
        toSerial.stamp = this->get_clock()->now();
        toSerial.id = static_cast<uint8_t>(10);
        auto payload = double2uint8Array(servo_angle);
        toSerial.payload.data.assign(payload.begin(), payload.end());
        serial_publisher_->publish(toSerial);
    }

    std::vector<uint8_t> double2uint8Array(double value)
    {
        // 1) scale and truncate
        int32_t scaled = static_cast<int32_t>(value);

        // 2) extract big-endian bytes (high byte first)
        uint8_t high = static_cast<uint8_t>((scaled >> 8) & 0xFF);
        uint8_t low = static_cast<uint8_t>(scaled & 0xFF);

        return {high, low};
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;

    // possible publisher for raw serial message
    uint8_t servo_id_;
    rclcpp::Publisher<sail_msgs::msg::SerialMsg>::SharedPtr serial_publisher_; // <-- FIXED TYPE

    // Subscriber for the wand position
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                         // Initialize ROS 2
    rclcpp::spin(std::make_shared<Wand2ServoNode>()); // Run the node
    rclcpp::shutdown();                               // Shut down ROS 2
    return 0;
}
