#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class RideHeightNode : public rclcpp::Node
{
public:
  RideHeightNode()
  : Node("ride_height_node"),
    L_wand_(declare_parameter<double>("L_wand", 0.60)),
    L_rod_ (declare_parameter<double>("L_rod", 1.10)),
    cov_z_ (declare_parameter<double>("variance_z", 0.0004))      // m²
  {
    wand_topic_ = declare_parameter<std::string>("wand_topic", "/wand_angle_deg");

    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
             "height_stamped", 10);

    wand_sub_ = create_subscription<std_msgs::msg::Float64>(
      wand_topic_, 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg)
      { wand_deg_ = msg->data; have_wand_ = true; });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu_data", 20,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg)
      {
        tf2::Quaternion q; tf2::fromMsg(msg->orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        roll_  = roll;
        pitch_ = pitch;
        stamp_ = msg->header.stamp;
        have_imu_ = true;
        compute_publish();
      });
  }

private:
  void compute_publish()
  {
    if (!have_wand_ || !have_imu_) return;

    const double wand_rad = wand_deg_ * M_PI / 180.0;
    const double height   = (L_wand_ * std::sin(wand_rad) -
                              L_rod_  * sin(pitch_)) * std::cos(roll_);

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp    = this->now();
    out.header.frame_id = "odom";

    // solo z
    out.pose.pose.position.z = height;
    out.pose.pose.orientation.w = 1.0;           // quaternion identità

    // covarianza: disattiva tutto tranne z–z
    std::fill(out.pose.covariance.begin(), out.pose.covariance.end(), 1e9);
    out.pose.covariance[2*6 + 2] = cov_z_;       // indice (row 2, col 2)

    pub_->publish(out);
  }

  /* parametri */
  double L_wand_, L_rod_, cov_z_;
  std::string wand_topic_;

  /* stato runtime */
  double wand_deg_{0.0}, roll_{0.0}, pitch_{0.0};
  rclcpp::Time stamp_;
  bool have_wand_{false}, have_imu_{false};

  /* ROS */
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wand_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr  imu_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RideHeightNode>());
  rclcpp::shutdown();
  return 0;
}