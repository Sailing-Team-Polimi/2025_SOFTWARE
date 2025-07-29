#include "rclcpp/rclcpp.hpp"
#include "BusInterface.hpp"  // Header containing CanPackage and SerialInterface
#include <system_error>

#include "sail_msgs/msg/serial_msg.hpp"

using std::placeholders::_1;

class BusNode : public rclcpp::Node {
public:
    BusNode()
    : Node("bus_node")
    {
        // Initialize SerialInterface singleton
        try {
            serial_ = &bus::SerialInterface::getInstance("/dev/ttyAMA0", B115200);
        } catch (const std::system_error &e) {
            RCLCPP_FATAL(get_logger(), "Failed to open serial port: %s", e.what());
            throw;
        }

        // Declare parameters with default values
        this->declare_parameter<int>("freq", 150);
        int timer_frequency_ = this->get_parameter("freq").as_int();
        int period_ms = static_cast<int>(1000.0 / timer_frequency_);

        // Create a timer to call getAll() at 1 Hz
        publisher_ = this->create_publisher<sail_msgs::msg::SerialMsg>("fromBrunilde", 10);

        // Timer callback using the timer frequency parameter (Hz)
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&BusNode::fromBrunilde, this)
        );

        subscription_ = this->create_subscription<sail_msgs::msg::SerialMsg>(
        "toBrunilde", 10, std::bind(&BusNode::toBrunilde, this, _1));
        
    }
    

private:
    /**
     * @brief Timer callback: request all packages and log them
     */
    void fromBrunilde()
    {
        auto packages = serial_->readAll();

        for (const auto &pkg : packages) {
            pkg.logPackage();

            sail_msgs::msg::SerialMsg msgFromSerial;

            msgFromSerial.stamp = this->get_clock()->now();

            auto id = pkg.getHeader();

            msgFromSerial.id = id[0]; // we give for granted we are using single byte header packages

            auto payload = pkg.getPayload();
            msgFromSerial.payload.data.assign(payload.begin(), payload.end());

            publisher_->publish(msgFromSerial);   
        }
    }
 
    void toBrunilde(const sail_msgs::msg::SerialMsg::SharedPtr msg) const
    {
        std::vector<uint8_t> header {msg->id};
        std::vector<uint8_t> payload(msg->payload.data.begin(), msg->payload.data.end());
        bus::CanPackage packageToSend = bus::CanPackage(header, payload);

        packageToSend.logPackage();
        serial_->send(packageToSend);
    }
    rclcpp::Subscription<sail_msgs::msg::SerialMsg>::SharedPtr subscription_;
    bus::SerialInterface *serial_;
    rclcpp::Publisher<sail_msgs::msg::SerialMsg>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BusNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
