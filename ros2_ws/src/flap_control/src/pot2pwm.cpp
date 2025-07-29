#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

using std::placeholders::_1;

class Pot2PwmNode : public rclcpp::Node
{
public:
  Pot2PwmNode()  // ← Nome costruttore corretto
  : Node("pot2pwm")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Int16>(
      "pot_data", 10, std::bind(&Pot2PwmNode::topic_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::Int16>("pwm_out", 10);
  }

private:
  // Coefficienti per le conversioni
  const float m1 = 180.0f / 4096.0f;
  const float m2 = 65535.0f / 180.0f;

  // Conversione: potenziometro 16 bit → angolo servo [-90, +90]
  float pot2servoAngle(int pot16bit)
  {
    return m1 * pot16bit - 90.0f;
  }

  // Conversione: angolo servo → PWM 16 bit (centered at 32767)
  int servoAngle2pwm(float servo_angle)
  {
    return static_cast<int>(m2 * servo_angle + 32767);
  }

  void topic_callback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    int pot_value = msg->data;
    float angle = pot2servoAngle(pot_value);
    int pwm = servoAngle2pwm(angle);

    RCLCPP_INFO(this->get_logger(), "🎛️ pot: %d -> angle: %.2f -> pwm: %d", pot_value, angle, pwm);

    std_msgs::msg::Int16 pwm_msg;
    pwm_msg.data = pwm;
    publisher_->publish(pwm_msg);
  }

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pot2PwmNode>());
  rclcpp::shutdown();
  return 0;
}
