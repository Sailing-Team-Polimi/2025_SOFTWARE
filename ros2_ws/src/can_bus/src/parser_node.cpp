#include "rclcpp/rclcpp.hpp"
#include "can_bus/parser/Parser.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Crea il nodo ROS
    auto node = std::make_shared<rclcpp::Node>("parser_node");

    // Istanzia il parser con il nodo
    auto parser = std::make_shared<can_bus::Parser>(node);

    // Crea un timer che chiama readAndDistribute() ogni 50ms
    auto timer = node->create_wall_timer(
        50ms, [parser]()
        { parser->readAndDistribute(); });

    // Avvia il nodo ROS
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
