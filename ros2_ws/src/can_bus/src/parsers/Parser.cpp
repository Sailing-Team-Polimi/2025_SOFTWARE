#include "can_bus/parser/Parser.hpp"
#include "can_bus/sensors/PotenziometerSensor.hpp"
#include <stdexcept>

namespace can_bus
{

    Parser::Parser(rclcpp::Node::SharedPtr node)
        : node_(node), serial_(bus::SerialInterface::getInstance())
    {

        std::vector<int64_t> ids = node_->get_parameter("ids").as_integer_array();
        std::vector<std::string> topics = node_->get_parameter("topics").as_string_array();

        if (ids.size() != topics.size())
        {
            RCLCPP_ERROR(node_->get_logger(), "Mismatch between 'ids' and 'topics' array sizes!");
            return;
        }

        for (size_t i = 0; i < ids.size(); ++i)
        {
            uint8_t id = static_cast<uint8_t>(ids[i]);
            const std::string &topic = topics[i];

            auto sensor = std::make_shared<PotenziometerSensor>(id, topic);
            sensors_.push_back(sensor);

            RCLCPP_INFO(node_->get_logger(), "Configured sensor ID %d on topic '%s'", id, topic.c_str());
        }
    }

    void Parser::addSensor(std::shared_ptr<Sensor> sensor)
    {
        sensors_.push_back(sensor);
    }

    void Parser::handlePacket(const bus::CanPackage &pkg)
    {
        bool handled = false;
        uint8_t id = pkg.getHeader()[0];

        for (auto &sensor : sensors_)
        {
            auto expected_ids = sensor->getExpectedIDs();
            if (std::find(expected_ids.begin(), expected_ids.end(), id) != expected_ids.end())
            {
                sensor->addPacket(pkg);

                if (sensor->isReady())
                {
                    sensor->publish(node_);
                    sensor->reset();
                }

                handled = true;
                break; // se un sensore ha gestito il pacchetto, possiamo uscire
            }
        }

        if (!handled)
        {
            std::ostringstream hex_payload;
            for (uint8_t byte : pkg.getPayload())
            {
                hex_payload << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            }

            RCLCPP_WARN(
                node_->get_logger(),
                "Discarded packet with unknown ID %d. Payload: [%s]",
                id,
                hex_payload.str().c_str());
        }
    }

    void Parser::readAndDistribute()
    {
        auto packets = serial_.readAll();
        for (const auto &pkg : packets)
        {
            handlePacket(pkg);
        }
    }

} // namespace can_bus
