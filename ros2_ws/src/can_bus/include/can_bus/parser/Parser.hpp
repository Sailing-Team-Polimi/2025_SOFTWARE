#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <cstdint>
#include "can_bus/sensors/Sensor.hpp"
#include "BusInterface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace can_bus
{

    class Parser
    {
    public:
        Parser(rclcpp::Node::SharedPtr node);

        // Aggiunge un sensore alla lista gestita
        void addSensor(std::shared_ptr<Sensor> sensor);

        // Processa un pacchetto ricevuto
        void handlePacket(const bus::CanPackage &pkg);

        // Esegue il ciclo completo: legge pacchetti e li distribuisce ai sensori
        void readAndDistribute();

    private:
        rclcpp::Node::SharedPtr node_;
        std::vector<std::shared_ptr<Sensor>> sensors_;
        bus::SerialInterface &serial_; // Singleton
    };

} // namespace can_bus
