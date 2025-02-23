#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class RPMController : public rclcpp::Node {
public:
    RPMController() : Node("rpm_controller") {
        // Declare and get parameters
        this->declare_parameter("wheelbase", 0.19);
        this->declare_parameter("wheel_radius", 0.09);
        this->declare_parameter("max_rpm", 40.0);

        wheelbase_ = this->get_parameter("wheelbase").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        max_rpm_ = this->get_parameter("max_rpm").as_double();

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&RPMController::cmdVelCallback, this, std::placeholders::_1)
        );

        left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/left_wheel_rpm", 10);
        right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right_wheel_rpm", 10);

        RCLCPP_INFO(this->get_logger(), "RPM Controller Node Started.");
    }

private:
    double wheelbase_, wheel_radius_, max_rpm_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_pub_, right_wheel_pub_;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double v = msg->linear.x;      // Linear velocity (m/s)
        double omega = msg->angular.z; // Angular velocity (rad/s)

        // Computing left & right wheel velocities (m/s)
        double v_left = v - (wheelbase_ / 2.0) * omega;
        double v_right = v + (wheelbase_ / 2.0) * omega;

        // Converting to RPM
        double rpm_left = (v_left / wheel_radius_) * (60.0 / (2.0 * M_PI));
        double rpm_right = (v_right / wheel_radius_) * (60.0 / (2.0 * M_PI));

        // Applying max RPM limit
        rpm_left = std::max(std::min(rpm_left, max_rpm_), -max_rpm_);
        rpm_right = std::max(std::min(rpm_right, max_rpm_), -max_rpm_);

        // Publishing RPM values
        std_msgs::msg::Float64 left_msg, right_msg;
        left_msg.data = rpm_left;
        right_msg.data = rpm_right;

        left_wheel_pub_->publish(left_msg);
        right_wheel_pub_->publish(right_msg);

        RCLCPP_INFO(this->get_logger(), "RPM Published: Left=%.2f, Right=%.2f", rpm_left, rpm_right);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RPMController>());
    rclcpp::shutdown();
    return 0;
}
