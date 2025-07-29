#pragma once

#include <vector>
#include <memory>
#include "can_bus/BusInterface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace can_bus
{

    class Sensor
    {
    public:
        virtual ~Sensor() = default;

        // Aggiunge un pacchetto ricevuto (es. con ID 5 per roll)
        virtual void addPacket(const bus::CanPackage &pkg) = 0;

        // Ritorna true se sono arrivati tutti i pacchetti attesi (es. roll, pitch, yaw, ax, ay, az)
        virtual bool isReady() const = 0;

        // Genera e ritorna un messaggio ROS da pubblicare (sensor_msgs::Imu, std_msgs::Float32, ecc.)
        virtual void publish(rclcpp::Node::SharedPtr node) = 0;

        // Pulisce i buffer interni (es. resetta le flag roll.pitch.yaw a false)
        virtual void reset() = 0;

        // Ritorna gli ID CAN che questo sensore si aspetta di ricevere
        virtual std::vector<uint8_t> getExpectedIDs() const = 0;
    };

} // namespace can_bus
