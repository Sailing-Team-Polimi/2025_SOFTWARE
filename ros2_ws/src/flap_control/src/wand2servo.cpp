#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"
#include <cmath>

#include "sail_msgs/msg/serial_msg.hpp"

class Wand2ServoNode : public rclcpp::Node
{
public:
    Wand2ServoNode() : Node("wand2servo")
    {
        this->declare_parameter("hc1", 5.5);
        this->declare_parameter("c1", 3.8);
        this->declare_parameter("b1", 14.0);
        this->declare_parameter("D1_1", 17.0);
        this->declare_parameter("a2", 2.0);
        this->declare_parameter("b2", 129.0);
        this->declare_parameter("c2", 2.2);
        this->declare_parameter("L_rod", 200.7);
        this->declare_parameter("L_foil", 129.0);
        this->declare_parameter("OA", 2.82);
        this->declare_parameter("Xoc", -200.7);
        this->declare_parameter("BC", 6.0);
        this->declare_parameter("CD", 1.9);
        this->declare_parameter("Yoc", -6.0);
        this->declare_parameter("EF", 2.2);

        this->declare_parameter<int>("servo_id", 10);

        publisher_ = this->create_publisher<std_msgs::msg::Float64>("servo_angle", 10);

        this->get_parameter("servo_id", servo_id_);

        if (servo_id_ != 0)
            serial_publisher_ = this->create_publisher<sail_msgs::msg::SerialMsg>("toSerial", 10);

        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "wand_angle", 10, std::bind(&Wand2ServoNode::wand_callback, this, std::placeholders::_1));

        load_constants_from_parameters();
        calculate_and_save();
    }

private:
    int servo_id_ = 0; // Default servo ID
    double hc1, c1, b1, D1_1, a2, b2, c2;
    double D, E, F, theta_2, A, B, C;
    double k1, k2, k3, k4, k5, k6, k7, k8, k9, k10;
    double Xa, Xb, beta, Yd, Ye, L_rod, L_foil, OA, Xoc, BC, CD, Yoc, Yof, EF;

    // Funzione per limitare i valori nel range [-1, 1]
    std::function<double(double)> clamp = [](double x)
    {
        return std::max(-1.0, std::min(1.0, x));
    };

    void load_constants_from_parameters()
    {
        this->get_parameter("hc1", hc1);
        this->get_parameter("c1", c1);
        this->get_parameter("b1", b1);
        this->get_parameter("D1_1", D1_1);
        this->get_parameter("a2", a2);
        this->get_parameter("b2", b2);
        this->get_parameter("c2", c2);
        this->get_parameter("L_rod", L_rod);
        this->get_parameter("L_foil", L_foil);
        this->get_parameter("OA", OA);
        this->get_parameter("Xoc", Xoc);
        this->get_parameter("BC", BC);
        this->get_parameter("CD", CD);
        this->get_parameter("Yoc", Yoc);
        this->get_parameter("EF", EF);

        // Log dei parametri per il debug
        RCLCPP_INFO(this->get_logger(), "EF: %f, BC: %f, OA: %f", EF, BC, OA);
    }

    double wand2flap(double wand_angle)
    {
        Xa = OA * cos(wand_angle - 3 * M_PI / 4);
        Xb = Xa - L_rod;
        beta = acos(clamp((Xb - Xoc) / BC)) - M_PI / 2; // Clamp per evitare valori fuori dominio
        Yd = CD * sin(beta) + Yoc;
        Ye = Yd - L_foil;
        double flap_angle = -asin(clamp((Ye - Yof) / EF)) + M_PI; // Clamp per evitare NaN
        return flap_angle;
    }

    double flap2servo(double flap_angle)
    {
        D = sin(flap_angle) + k9;
        E = cos(flap_angle) + k7;
        F = k6 + k8 * cos(flap_angle) + k10 * sin(flap_angle);

        theta_2 = -asin(clamp(F / sqrt(D * D + E * E))) + atan2(E, D) + M_PI + 0.6; // Clamp per evitare NaN

        A = sin(theta_2) - k5;
        B = cos(theta_2) - k3;
        C = k1 - k2 * cos(theta_2) - k4 * sin(theta_2);

        return asin(clamp(C / sqrt(A * A + B * B))) - atan2(B, A); // Clamp per evitare NaN
    }

    void wand_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double wand_angle = msg->data;

        // Log dei valori intermedi per il debug
        RCLCPP_INFO(this->get_logger(), "Received wand angle: %.3f", wand_angle);

        double flap_angle = wand2flap(wand_angle);
        double servo_angle = flap2servo(flap_angle);

        // Log dei calcoli intermedi
        RCLCPP_INFO(this->get_logger(), "Flap angle: %.3f, Servo angle: %.3f", flap_angle * 180 / M_PI, servo_angle * 180 / M_PI);

        // Testing remapping
        float m = 1;
        float q = 45;
        servo_angle = m * wand_angle + q;

        auto new_msg = std_msgs::msg::Float64();
        new_msg.data = servo_angle * 180 / M_PI;

        // Log del risultato finale
        RCLCPP_INFO(this->get_logger(), "Published servo angle: %.3f", new_msg.data);

        publisher_->publish(new_msg);

        if (servo_id_ != 0)
        {
            auto toSerial = sail_msgs::msg::SerialMsg();
            toSerial.stamp = this->get_clock()->now();
            toSerial.id = static_cast<uint8_t>(servo_id_);
            auto payload = double2uint8Array(servo_angle * 180 / M_PI);
            toSerial.payload.data.assign(payload.begin(), payload.end());
            serial_publisher_->publish(toSerial);
        }
    }

    std::vector<uint8_t> double2uint8Array(double value)
    {
        // 1) scale and truncate
        int32_t scaled = static_cast<int32_t>(value);

        // 2) extract big-endian bytes (high byte first)
        uint8_t high = static_cast<uint8_t>((scaled >> 8) & 0xFF);
        uint8_t low = static_cast<uint8_t>(scaled & 0xFF);

        return {high, low};
    }

    void calculate_and_save()
    {
        double D2_1 = hc1 - c1;
        double a1 = sqrt(1.3 * 1.3 + hc1 * hc1);
        double L1_1 = sqrt(D1_1 * D1_1 + D2_1 * D2_1);
        double L2_1 = a1;
        double L3_1 = b1;
        double L4_1 = c1;
        double a = L2_1, b = L3_1, c = L4_1, d = L1_1;
        k1 = (a * a + c * c + d * d - b * b) / (2 * a * c);
        k2 = D1_1 / c;
        k3 = D1_1 / a;
        k4 = D2_1 / c;
        k5 = D2_1 / a;

        double H1 = c2 + a2;
        double H2 = -b2;
        double L5 = sqrt(H1 * H1 + H2 * H2);
        double L6 = a2, L7 = b2, L8 = c2;
        double e = L6, f = L7, g = L8, h = L5;
        k6 = (e * e + g * g + h * h - f * f) / (2 * e * g);
        k7 = H1 / g;
        k8 = H1 / e;
        k9 = H2 / g;
        k10 = H2 / e;
        Yof = Yoc - L_foil;
    }

    // possible publisher for raw serial message
    // uint8_t servo_id_;
    // rclcpp::Publisher<sail_msgs::msg::SerialMsg>::SharedPtr serial_publisher_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Publisher<sail_msgs::msg::SerialMsg>::SharedPtr serial_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Wand2ServoNode>());
    rclcpp::shutdown();
    return 0;
}
