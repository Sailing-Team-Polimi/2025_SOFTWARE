#include <chrono>
#include <memory>
#include <vector>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sail_msgs/msg/serial_msg.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define DATA_NUM 3
#define DATA_SIZE 2
#define PACK_SIZE DATA_NUM*DATA_SIZE


// Structure to hold serial data for one field.
struct SerialDataManager {
  int h;         // Expected ID value.
  std::vector<double> data;   // Scaled data.
  bool flag;     // Indicates if new data has been received.
  SerialDataManager() : h(0), data(0.0), flag(false) {}
};

// Holds IMU covariance values.
struct IMUManager {
  std::array<double, 9> rpy_cov;
  std::array<double, 9> vel_cov;
  std::array<double, 9> acc_cov;


    IMUManager( double rpy_dev2   = 0.01,
                double vel_dev2   = 0.1,
                double acc_dev2   = 5.0   ) {

    double diag = 0.0;

    rpy_cov = {rpy_dev2, diag, diag,
               diag, rpy_dev2, diag,
               diag, diag, rpy_dev2};
    
    vel_cov = {vel_dev2, diag, diag,
               diag, vel_dev2, diag,
               diag, diag, vel_dev2};

    acc_cov = {acc_dev2, diag, diag,
                diag, acc_dev2, diag,
                diag, diag, acc_dev2};
  }
};

class IMUParser : public rclcpp::Node {
public:
  IMUParser() : Node("imu_parser") {
    // Declare parameters.
    this->declare_parameter<std::string>("link", "imu_link");
    this->declare_parameter<std::vector<int64_t>>("id", std::vector<int64_t>{2,3,4});
    this->declare_parameter<std::vector<double>>("tf", std::vector<double>{0.0, 0.0});
    this->declare_parameter<std::vector<double>>("cov", std::vector<double>{0.01, 0.1, 5.0}); // | rpi | vel | acc | covariances
    this->declare_parameter<bool>("publish_pose", false);
    this->declare_parameter<bool>("publish_ypr", false);

    std::vector<double> cov;
    this->get_parameter("cov", cov);
    if (cov.size() >= 3) {
      imu_manager_ = IMUManager(cov[0], cov[1], cov[2]);
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter 'rpy_cov' or 'acc_cov does not have 3 elements. Using defaults.");
      imu_manager_ = IMUManager();
    }

    // Retrieve parameters.
    this->get_parameter("link", link_);
    std::vector<int64_t> id_vector;
    this->get_parameter("id", id_vector);

    if (id_vector.size() == 3) {
      rpy_.h  = id_vector[0];
      vel_.h  = id_vector[1];
      acc_.h = id_vector[2];
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter 'id' does not have 3 elements. Using defaults.");
      rpy_.h = 2; vel_.h = 3; acc_.h = 4;
    }

    std::vector<double> tf_vector;
    this->get_parameter("tf", tf_vector);
    if (tf_vector.size() == 6) { // If TF parameter is setup correctly, publish tf

      tf_vector[3] = tf_vector[3] * (M_PI / 180.0);
      tf_vector[4] = tf_vector[4] * (M_PI / 180.0);
      tf_vector[5] = tf_vector[5] * (M_PI / 180.0);
      tf_ = tf_vector;
      // Initialize the transform broadcaster.
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
      // Timer for publishing the transform at regular intervals.
      tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&IMUParser::publish_transform, this));

    } else { // if not, avoid it 
      RCLCPP_WARN(this->get_logger(), "Parameter 'tf' does not have 6 elements. Avoid publishing tf.");
      
    }

    // Create subscription to serial data.
    serial_sub_ = this->create_subscription<sail_msgs::msg::SerialMsg>(
      "serial_data", 10, std::bind(&IMUParser::msg_callback, this, std::placeholders::_1));
    // Create publishers for IMU and RPY messages.
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);

    this->get_parameter("publish_pose", publish_pose_);
    if (publish_pose_) {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("imu_pose", 10);
    }

    this->get_parameter("publish_ypr", publish_ypr_);
    if (publish_ypr_) {
        ypr_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("imu_ypr", 10);
    }
    
  }

private:
  // Process incoming serial messages.
  void msg_callback(const sail_msgs::msg::SerialMsg::SharedPtr msg) {
    int msg_id = static_cast<int>(msg->id);
    int msg_size = static_cast<int>(msg->payload.data.size());

    // 1) Filter by ID
    if (msg_id != static_cast<uint8_t>(rpy_.h) || msg_id != static_cast<uint8_t>(vel_.h) || msg_id != static_cast<uint8_t>(acc_.h)) {
      return;
    }

    // 2) Check payload length
    if (msg_size != static_cast<size_t>(PACK_SIZE)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Unexpected payload size %zu (expected %d)",
        msg->payload.data.size(), PACK_SIZE);
      return;
    }

    // Scale angular values (roll, pitch, yaw) by 0.01*(pi/180) and acceleration by 0.01.
    if (msg_id == rpy_.h) {
      std::vector<int64_t> raw_data = convertBytesToInt64Array(msg->payload.data, DATA_SIZE, DATA_NUM);
      for(int  i=0;i<DATA_NUM; i++){ rpy_.data[i]= raw_data[i]* 0.01 * (M_PI / 180.0); }
      rpy_.flag = true;
    }

    if (msg_id == vel_.h) {
      std::vector<int64_t> raw_data = convertBytesToInt64Array(msg->payload.data, DATA_SIZE, DATA_NUM);
      for(int  i=0;i<DATA_NUM; i++){ vel_.data[i]= raw_data[i]* 0.01; }
      vel_.flag = true;
    }
    
    if (msg_id == acc_.h) {
      std::vector<int64_t> raw_data = convertBytesToInt64Array(msg->payload.data, DATA_SIZE, DATA_NUM);
      for(int  i=0;i<DATA_NUM; i++){ acc_.data[i]= raw_data[i]* 0.01; }
      acc_.flag = true;
    } 

    // When all fields are received, publish the IMU and RPY messages.
    if (rpy_.flag && vel_.flag && acc_.flag) {
        auto imu_msg = create_imu_msg();
        imu_pub_->publish(imu_msg);

        if(publish_pose_) {
            auto pose_msg = create_pose_msg();
            pose_pub_->publish(pose_msg);
        }

        if(publish_ypr_) {
          auto ypr_msg = create_ypr_msg();
          ypr_pub_->publish(ypr_msg);
        }

        // Reset flags for the next set.
        rpy_.flag = acc_.flag = vel_.flag = false;
    }
  }

  // Create an IMU message from the collected data.
  sensor_msgs::msg::Imu create_imu_msg() {
    sensor_msgs::msg::Imu msg;
    double roll = rpy_.data[0];
    double pitch = rpy_.data[1];
    double yaw = rpy_.data[2];
    // Use tf2 to create a quaternion from roll, pitch, yaw.
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    msg.orientation = tf2::toMsg(q);

    for (size_t i = 0; i < imu_manager_.rpy_cov.size(); i++) {
      msg.orientation_covariance[i] = imu_manager_.rpy_cov[i];
    }
    msg.linear_acceleration.x = acc_.data[0];
    msg.linear_acceleration.y = acc_.data[1];
    msg.linear_acceleration.z = acc_.data[2];
    for (size_t i = 0; i < imu_manager_.acc_cov.size(); i++) {
      msg.linear_acceleration_covariance[i] = imu_manager_.acc_cov[i];
    }
    // Set frame_id from the 'link' parameter.
    msg.header.frame_id = link_;
    msg.header.stamp = this->now();
    return msg;
  }

  // Create a raw RPY (roll, pitch, yaw) message.
  geometry_msgs::msg::PoseWithCovarianceStamped create_pose_msg() {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = link_;
    
    // Convert roll, pitch, yaw to quaternion using tf2:
    tf2::Quaternion q;
    double roll = rpy_.data[0];
    double pitch = rpy_.data[1];
    double yaw = rpy_.data[2];
    q.setRPY(roll, pitch, yaw);
    msg.pose.pose.orientation = tf2::toMsg(q);
    
    // Set position to zero (or modify as needed)
    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;
    msg.pose.pose.position.z = 0.0;
    
    // Initialize the 6x6 covariance matrix (36 elements) to zero.
    for (size_t i = 0; i < 36; i++) {
      msg.pose.covariance[i] = 0.0;
    }
    
    // Assuming imu_manager_.rpy_cov is a 3x3 matrix (row-major) with:
    // [roll_variance, 0, 0, 0, pitch_variance, 0, 0, 0, yaw_variance],
    // assign these to the corresponding positions for orientation covariance:
    msg.pose.covariance[21] = imu_manager_.rpy_cov[0]; // roll variance (rot about X)
    msg.pose.covariance[28] = imu_manager_.rpy_cov[4]; // pitch variance (rot about Y)
    msg.pose.covariance[35] = imu_manager_.rpy_cov[8]; // yaw variance (rot about Z)
    
    return msg;
  }

  // Create a raw RPY (roll, pitch, yaw) message.
  geometry_msgs::msg::Vector3 create_ypr_msg() {
    
    geometry_msgs::msg::Vector3 msg;
    double roll = rpy_.data[0];
    double pitch = rpy_.data[1];
    double yaw = rpy_.data[2];

    msg.x = roll * (180.0 / M_PI);
    msg.y = pitch * (180.0 / M_PI);
    msg.z = yaw * (180.0 / M_PI);

    return msg;
  }


  // Publish a transform from "base_link" to the link frame.
  void publish_transform() {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = link_;
    // Translation: first three elements of tf parameter.
    transformStamped.transform.translation.x = tf_[0];
    transformStamped.transform.translation.y = tf_[1];
    transformStamped.transform.translation.z = tf_[2];
    // Orientation: tf is [x, y, z, yaw, pitch, roll]
    double roll = tf_[3];
    double pitch = tf_[4];
    double yaw = tf_[5];
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    transformStamped.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(transformStamped);
  }

  // Function to convert a vector of bytes into a vector of int64_t values.
  // The conversion assumes that the vector's size is exactly data_number * byte_size,
  // and that the integer values are stored in big-endian order starting from the last byte
  // of the vector.
  std::vector<int64_t> convertBytesToInt64Array(
    const std::vector<uint8_t>& data,
    int byte_size,
    int data_number)
  {
    // Optionally, check if the data size is as expected.
    size_t expected_size = static_cast<size_t>(byte_size) * static_cast<size_t>(data_number);
    if (data.size() != expected_size) {
        std::cerr << "Warning: expected " << expected_size << " bytes, but got " << data.size() << " bytes.\n";
    }

    std::vector<int64_t> result(data_number);
    size_t total_size = data.size();

    // Process each data group.
    // The first integer is built from the last 'byte_size' bytes.
    for (int i = 0; i < data_number; i++) {
        // Compute the starting index: for i == 0, this will be total_size - byte_size.
        size_t start_index = total_size - static_cast<size_t>(i + 1) * byte_size;
        // Convert this slice of bytes using the combineBytesBigEndian function.
        uint64_t data_i = combineBytesBigEndian(&data[start_index], byte_size);
        result[i] = static_cast<int64_t>(data_i);
    }
    return result;
  }
  
  uint64_t combineBytesBigEndian(const uint8_t* bytes, size_t length) {
    uint64_t result = 0;
    for (size_t i = 0; i < length; ++i) {
        result = (result << 8) | bytes[i];
    }
    return result;
  }

  // ROS 2 communication objects.
  rclcpp::Subscription<sail_msgs::msg::SerialMsg>::SharedPtr serial_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr ypr_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  // Data managers for each measurement.
  SerialDataManager rpy_;
  SerialDataManager vel_;
  SerialDataManager acc_;

  IMUManager imu_manager_;

  // Parameters.
  std::string link_;
  std::vector<double> tf_;
  bool publish_pose_;
  bool publish_ypr_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IMUParser>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
