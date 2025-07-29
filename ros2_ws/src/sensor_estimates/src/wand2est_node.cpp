#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;

class wand2est : public rclcpp::Node
{
public:
wand2est() : Node("state_estimator_node") {
    // Subscriber al sensore us 
    wand_sub_ = this->create_subscription<std_msgs::msg::Float32>("wand_angle", 10,
      std::bind(&wand2est::wand_callback, this, _1));
    // Subscriber al sensore imu, mi da quaternione, accelerazione e velocità anglare, voglio solo il quaternione
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu_data", 10,
      std::bind(&wand2est::imu_callback, this, _1));
    // Publisher per l'altezza stimata
    est_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("height_est_wand", 10);
    
    est_msg_.header.frame_id = "base_link";
    est_msg_.header.stamp = this->now();
    est_msg_.pose.covariance[14] = 0.01; // Covariance for z

    this->declare_parameter("d", 1.74); // centimetres
    this->get_parameter("d", d_);
    
    this->declare_parameter("wand_length", 1.05); // centimetres
    this->get_parameter("wand_length", wand_length_);

    timer = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&wand2est::est_pub_callback, this));
  }
  private:

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // extract the orientation from the IMU message
    auto& q_msg = msg->orientation;

    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    roll_ = roll;
    pitch_ = pitch;

  }

  void wand_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    // Process the wand data here
    auto wand_angle = msg->data;
    wand_angle_ = wand_angle;
  }

  void est_pub_callback()
  {
    // Create a PoseWithCovarianceStamped message
    est_msg_.header.stamp = this->now();
    est_msg_.pose.pose.position.z = getZest();
    est_pub_->publish(est_msg_);

  }

  double getZest()
  {
    // Compute the estimated height based on US data and IMU orientation
    // For example, you can use a simple model or a Kalman filter
    // Here, we just return the US data for simplicity
    double wand_angle = wand_angle_; // distance from us to the center of the robot
    double roll = roll_;
    double pitch = pitch_;

    double z_est = (wand_length_*sin(wand_angle_) + d_*sin(pitch)) * cos (roll);

    return z_est;
  }
  
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wand_sub_;
  double wand_angle_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  double roll_, pitch_;
  
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr est_pub_;
  geometry_msgs::msg::PoseWithCovarianceStamped est_msg_;
  std::string frame_id_;

  rclcpp::TimerBase::SharedPtr timer;

  double d_;
  double wand_length_;
};

int main(int argc, char * argv[])
{
    // Initialize the ROS 2 C++ client library
    rclcpp::init(argc, argv);
    // Create an instance of the us2est class and runs it continuously
    rclcpp::spin(std::make_shared<wand2est>());
    // Shutdown the ROS 2 C++ client library
    rclcpp::shutdown();
    return 0;
}
