#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo
{
    class GazeboRPMPlugin : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
        {
            // ROS2 Node
            node_ = gazebo_ros::Node::Get(sdf);

            // Get wheel joints
            left_wheel_joint_ = model->GetJoint("base_left_wheel_joint");
            right_wheel_joint_ = model->GetJoint("base_right_wheel_joint");

            if (!left_wheel_joint_ || !right_wheel_joint_)
            {
                RCLCPP_ERROR(node_->get_logger(), "Wheel joints not found!");
                return;
            }

            // Subscribe to RPM topics
            left_wheel_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
                "/left_wheel_rpm", 10,
                std::bind(&GazeboRPMPlugin::LeftRPMCallback, this, std::placeholders::_1));

            right_wheel_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
                "/right_wheel_rpm", 10,
                std::bind(&GazeboRPMPlugin::RightRPMCallback, this, std::placeholders::_1));

            RCLCPP_INFO(node_->get_logger(), "Gazebo RPM Plugin Loaded.");
        }

    private:
        gazebo_ros::Node::SharedPtr node_;
        physics::JointPtr left_wheel_joint_, right_wheel_joint_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_wheel_sub_, right_wheel_sub_;

        void LeftRPMCallback(const std_msgs::msg::Float64::SharedPtr msg)
        {
            double rad_per_sec = (msg->data * 2.0 * M_PI) / 60.0; // Convert RPM to rad/s
            left_wheel_joint_->SetVelocity(0, rad_per_sec);
        }

        void RightRPMCallback(const std_msgs::msg::Float64::SharedPtr msg)
        {
            double rad_per_sec = (msg->data * 2.0 * M_PI) / 60.0; // Convert RPM to rad/s
            right_wheel_joint_->SetVelocity(0, rad_per_sec);
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(GazeboRPMPlugin)
}
