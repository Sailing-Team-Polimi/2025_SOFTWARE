#include <chrono>
#include <memory>
#include <array>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sail_msgs/msg/serial_msg.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ---- Protocol constants -----------------------------------------------------
static constexpr int DATA_NUM  = 3;   // R, P, Y / wx, wy, wz / ax, ay, az
static constexpr int DATA_SIZE = 2;   // bytes per element (int16, big-endian)
static constexpr int PACK_SIZE = DATA_NUM * DATA_SIZE; // bytes per packet

// ---- Small helpers ----------------------------------------------------------
inline int16_t be16(const uint8_t *p) {
  // combine two big-endian bytes into signed 16-bit
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

struct SerialData {
  int id{0};
  std::array<double, DATA_NUM> v{0.0, 0.0, 0.0};
  bool received{false};
};

// Holds IMU covariance values.
struct IMUManager {
  std::array<double, 9> rpy_cov{};
  std::array<double, 9> vel_cov{};  // kept for completeness
  std::array<double, 9> acc_cov{};

  IMUManager(double rpy_dev2 = 0.01, double vel_dev2 = 0.1, double acc_dev2 = 5.0) {
    const double d = 0.0;
    rpy_cov = {rpy_dev2, d, d, d, rpy_dev2, d, d, d, rpy_dev2};
    vel_cov = {vel_dev2, d, d, d, vel_dev2, d, d, d, vel_dev2};
    acc_cov = {acc_dev2, d, d, d, acc_dev2, d, d, d, acc_dev2};
  }
};

class IMUParser : public rclcpp::Node {
public:
  IMUParser() : Node("imu_parser", rclcpp::NodeOptions().use_intra_process_comms(true)) {
    // Parameters
    this->declare_parameter<std::string>("link", "imu_link");
    this->declare_parameter<std::vector<int64_t>>("id", {2, 3, 4}); // rpy, vel, acc
    this->declare_parameter<std::vector<bool>>("available", {true, false, true});
    this->declare_parameter<std::vector<double>>("tf", {0.0, 0.0, 0.0, /*roll*/0.0, /*pitch*/0.0, /*yaw*/0.0});
    this->declare_parameter<std::vector<double>>("cov", {0.01, 0.1, 5.0}); // rpy, vel, acc variances
    this->declare_parameter<bool>("publish_pose", false);
    this->declare_parameter<bool>("publish_ypr", false);
    this->declare_parameter<double>("tf_publish_hz", 10.0);

    // pull params
    this->get_parameter("link", link_);

    std::vector<int64_t> ids; this->get_parameter("id", ids);
    if (ids.size() == 3) { rpy_.id = ids[0]; vel_.id = ids[1]; acc_.id = ids[2]; }

    std::vector<bool> avail; this->get_parameter("available", avail);
    if (avail.size() != 3) avail = {true, false, true};
    available_rpy_ = avail[0];
    available_vel_ = avail[1];
    available_acc_ = avail[2];

    std::vector<double> cov; this->get_parameter("cov", cov);
    if (cov.size() >= 3) imu_manager_ = IMUManager(cov[0], cov[1], cov[2]);

    std::vector<double> tfv; this->get_parameter("tf", tfv);
    if (tfv.size() == 6) {
    // Expect tf = [x,y,z, roll(deg), pitch(deg), yaw(deg)]
    tf_[0] = tfv[0];
    tf_[1] = tfv[1];
    tf_[2] = tfv[2];
    const double deg2rad = M_PI / 180.0;
    tf_[3] = tfv[3] * deg2rad;
    tf_[4] = tfv[4] * deg2rad;
    tf_[5] = tfv[5] * deg2rad;


    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / std::max(1e-3, this->get_parameter("tf_publish_hz").as_double())));
    tf_timer_ = this->create_wall_timer(period, std::bind(&IMUParser::publish_transform, this));
    } else {
    RCLCPP_WARN(get_logger(), "Parameter 'tf' does not have 6 elements. TF won't be published.");
    }

    this->get_parameter("publish_pose", publish_pose_);
    this->get_parameter("publish_ypr", publish_ypr_);

    // QoS: sensor data profile for latency-sensitive streams
    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    serial_sub_ = this->create_subscription<sail_msgs::msg::SerialMsg>(
      "serial_data", qos, std::bind(&IMUParser::on_serial, this, std::placeholders::_1));

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", qos);
    if (publish_pose_)  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("imu_pose", qos);
    if (publish_ypr_)   ypr_pub_  = this->create_publisher<geometry_msgs::msg::Vector3>("imu_ypr", qos);
  }

private:
  // return true if message id matches any configured id
  inline bool id_accepts(int id) const {
    return id == rpy_.id || id == vel_.id || id == acc_.id;
  }

  void on_serial(const sail_msgs::msg::SerialMsg::SharedPtr msg) {
    const int id = static_cast<int>(msg->id);
    const size_t sz = msg->payload.data.size();

    // 1) Filter by ID (correct logical condition)
    if (!id_accepts(id)) return;

    // 2) Payload length check
    if (sz != static_cast<size_t>(PACK_SIZE)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Unexpected payload size %zu (expected %d)", sz, PACK_SIZE);
      return;
    }

    const uint8_t *p = msg->payload.data.data();
    std::array<double, DATA_NUM> tmp{};

    if (id == rpy_.id && available_rpy_) {
      // RPY coming as centi-degrees -> rad: 0.01 * deg2rad
      constexpr double k = 0.01 * M_PI / 180.0;
      tmp[0] = static_cast<double>(be16(&p[0])) * k;
      tmp[1] = static_cast<double>(be16(&p[2])) * k;
      tmp[2] = static_cast<double>(be16(&p[4])) * k;
      rpy_.v = tmp; rpy_.received = true;
    } else if (id == vel_.id && available_vel_) {
      // Angular velocity in 0.01 units -> SI rad/s scale as needed
      constexpr double k = 0.01; // adjust if your sensor differs
      tmp[0] = static_cast<double>(be16(&p[0])) * k;
      tmp[1] = static_cast<double>(be16(&p[2])) * k;
      tmp[2] = static_cast<double>(be16(&p[4])) * k;
      vel_.v = tmp; vel_.received = true;
    } else if (id == acc_.id && available_acc_) {
      // Linear acceleration in 0.01 m/s^2
      constexpr double k = 0.01;
      tmp[0] = static_cast<double>(be16(&p[0])) * k;
      tmp[1] = static_cast<double>(be16(&p[2])) * k;
      tmp[2] = static_cast<double>(be16(&p[4])) * k;
      acc_.v = tmp; acc_.received = true;
    }

    // Decide when we have enough to publish.
    const bool have_rpy = !available_rpy_ || rpy_.received; // if not available, treat as satisfied
    const bool have_acc = !available_acc_ || acc_.received;
    // vel is currently unused by sensor_msgs/Imu fields we fill (orientation + lin acc);
    // keep it optional but you can gate on it if you later use angular_velocity.

    if (have_rpy && have_acc) {
      auto imu = make_imu();
      imu_pub_->publish(imu);

      if (publish_pose_) {
        auto pose = make_pose();
        pose_pub_->publish(pose);
      }
      if (publish_ypr_) {
        auto ypr = make_ypr();
        ypr_pub_->publish(ypr);
      }

      // reset only those we actually require
      if (available_rpy_) rpy_.received = false;
      if (available_acc_) acc_.received = false;
      if (available_vel_) vel_.received = false;
    }
  }

  sensor_msgs::msg::Imu make_imu() {
    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = link_;
    msg.header.stamp = this->now();

    // orientation from rpy (or zero if not provided)
    tf2::Quaternion q;
    const double roll  = available_rpy_ ? rpy_.v[0] : 0.0;
    const double pitch = available_rpy_ ? rpy_.v[1] : 0.0;
    const double yaw   = available_rpy_ ? rpy_.v[2] : 0.0;
    q.setRPY(roll, pitch, yaw);
    msg.orientation = tf2::toMsg(q);
    for (size_t i = 0; i < 9; ++i) msg.orientation_covariance[i] = imu_manager_.rpy_cov[i];

    // angular velocity (optional if you later use it)
    if (available_vel_ && vel_.received) {
      msg.angular_velocity.x = vel_.v[0];
      msg.angular_velocity.y = vel_.v[1];
      msg.angular_velocity.z = vel_.v[2];
      for (size_t i = 0; i < 9; ++i) msg.angular_velocity_covariance[i] = imu_manager_.vel_cov[i];
    } else {
      // Per REP-145, set covariance[0] = -1 to indicate "no estimate"
      msg.angular_velocity_covariance[0] = -1.0;
    }

    // linear acceleration
    if (available_acc_) {
      msg.linear_acceleration.x = acc_.v[0];
      msg.linear_acceleration.y = acc_.v[1];
      msg.linear_acceleration.z = acc_.v[2];
      for (size_t i = 0; i < 9; ++i) msg.linear_acceleration_covariance[i] = imu_manager_.acc_cov[i];
    } else {
      msg.linear_acceleration_covariance[0] = -1.0;
    }

    return msg;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped make_pose() {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = link_;

    tf2::Quaternion q;
    const double roll  = available_rpy_ ? rpy_.v[0] : 0.0;
    const double pitch = available_rpy_ ? rpy_.v[1] : 0.0;
    const double yaw   = available_rpy_ ? rpy_.v[2] : 0.0;
    q.setRPY(roll, pitch, yaw);
    msg.pose.pose.orientation = tf2::toMsg(q);

    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;
    msg.pose.pose.position.z = 0.0;

    for (double &c : msg.pose.covariance) c = 0.0;
    msg.pose.covariance[21] = imu_manager_.rpy_cov[0];
    msg.pose.covariance[28] = imu_manager_.rpy_cov[4];
    msg.pose.covariance[35] = imu_manager_.rpy_cov[8];
    return msg;
  }

  geometry_msgs::msg::Vector3 make_ypr() {
    geometry_msgs::msg::Vector3 msg;
    const double roll  = available_rpy_ ? rpy_.v[0] : 0.0;
    const double pitch = available_rpy_ ? rpy_.v[1] : 0.0;
    const double yaw   = available_rpy_ ? rpy_.v[2] : 0.0;
    msg.x = roll  * (180.0 / M_PI);
    msg.y = pitch * (180.0 / M_PI);
    msg.z = yaw   * (180.0 / M_PI);
    return msg;
  }

  void publish_transform() {
    if (!tf_broadcaster_) return;
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "base_link"; // change via parameter if needed
    t.child_frame_id = link_;
    t.transform.translation.x = tf_[0];
    t.transform.translation.y = tf_[1];
    t.transform.translation.z = tf_[2];
    tf2::Quaternion q; q.setRPY(tf_[3], tf_[4], tf_[5]);
    t.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(t);
  }

  // --- members ---------------------------------------------------------------
  rclcpp::Subscription<sail_msgs::msg::SerialMsg>::SharedPtr serial_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr ypr_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  SerialData rpy_{};
  SerialData vel_{};
  SerialData acc_{};
  IMUManager imu_manager_{};

  std::string link_;
  std::array<double, 6> tf_{}; // [x,y,z, roll, pitch, yaw] in SI (rad)

  bool publish_pose_{false};
  bool publish_ypr_{false};
  bool available_rpy_{true};
  bool available_vel_{false};
  bool available_acc_{true};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUParser>());
  rclcpp::shutdown();
  return 0;
}
