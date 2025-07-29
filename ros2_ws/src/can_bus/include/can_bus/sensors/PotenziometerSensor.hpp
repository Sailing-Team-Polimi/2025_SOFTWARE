#pragma once

#include "Sensor.hpp"
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float32.hpp>
#include <optional>
#include <string>
#include <cstdint>

namespace can_bus
{

    class PotenziometerSensor : public Sensor
    {
    public:
        PotenziometerSensor(uint8_t expected_id, const std::string &topic_name);

        void addPacket(const bus::CanPackage &pkg) override;
        bool isReady() const override;
        void publish(rclcpp::Node::SharedPtr node) override;
        void reset() override;
        std::vector<uint8_t> getExpectedIDs() const override;

    private:
        uint8_t expected_id_;
        std::string topic_name_;
        std::optional<float> current_angle_; // valore mappato in gradi
    };

} // namespace can_bus
