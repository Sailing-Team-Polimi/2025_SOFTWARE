#include "can_bus/sensors/PotenziometerSensor.hpp"
#include "BusInterface.hpp"
#include <stdexcept>

namespace can_bus
{

    PotenziometerSensor::PotenziometerSensor(uint8_t expected_id, const std::string &topic_name)
        : expected_id_(expected_id), topic_name_(topic_name), current_angle_(std::nullopt) {}

    void PotenziometerSensor::addPacket(const bus::CanPackage &pkg)
    {
        if (pkg.getHeader().size() > 0 && pkg.getHeader()[0] == expected_id_)
        {
            try
            {
                int64_t raw_value = pkg.payloadAsInt();
                float angle = static_cast<float>(raw_value) * 90.0f / 4096.0f;
                current_angle_ = angle;
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(rclcpp::get_logger("PotenziometerSensor"), "Parsing failed: %s", e.what());
            }
        }
    }

    bool PotenziometerSensor::isReady() const
    {
        return current_angle_.has_value();
    }

    void PotenziometerSensor::publish(rclcpp::Node::SharedPtr node)
    {
        if (!isReady())
            return;

        auto pub = node->create_publisher<std_msgs::msg::Float32>(topic_name_, 10);
        std_msgs::msg::Float32 msg;
        msg.data = current_angle_.value();
        pub->publish(msg);

        RCLCPP_INFO(node->get_logger(), "Published potenziometer angle: %.2f", msg.data);
    }

    void PotenziometerSensor::reset()
    {
        current_angle_ = std::nullopt;
    }

    std::vector<uint8_t> PotenziometerSensor::getExpectedIDs() const
    {
        return {expected_id_};
    }

} // namespace can_bus
