#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/set_initial_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <iostream>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using SetInitialPose = nav2_msgs::srv::SetInitialPose;
using namespace std;

class WaypointNavigator : public rclcpp::Node {
public:
    WaypointNavigator() : Node("waypoint_navigator") {
        action_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose", action_callback_group_);
        client_service = this->create_client<SetInitialPose>("set_initial_pose");

        // Get package share directory and load waypoints
        std::string package_path = ament_index_cpp::get_package_share_directory("bot_world");
        std::string yaml_path = package_path + "/maps/waypoints.yaml";
        load_waypoints(yaml_path);

        // Wait for the action server
        while (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for action server...");
        }


        // Start waypoint selection loop
        set_initial_pose();
        select_and_navigate();
    }

private:
    struct Waypoint {
        double x, y, yaw;
    };

    std::map<std::string, Waypoint> waypoints_; // Stores waypoints by name
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    rclcpp::Client<SetInitialPose>::SharedPtr client_service;
    rclcpp::CallbackGroup::SharedPtr action_callback_group_;

    void set_initial_pose(){
        auto request= std::make_shared<SetInitialPose::Request>();
        if(!client_service->wait_for_service(std::chrono::seconds(4))){
            RCLCPP_WARN(this->get_logger(), "Waiting for service server...");
        }
        request->pose.header.stamp = this->get_clock()->now();
        request->pose.header.frame_id = "map";


        request->pose.pose.pose.position.x = 0.0;
        request->pose.pose.pose.position.y = 0.0;
        request->pose.pose.pose.position.z = 0.0;

        request->pose.pose.pose.orientation.x = 0.0;
        request->pose.pose.pose.orientation.y = 0.0;
        request->pose.pose.pose.orientation.z = 0.0;
        request->pose.pose.pose.orientation.w = 1.0;

        using ServiceResponseFuture = rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedFuture;
        auto future = client_service->async_send_request(request, std::bind(&WaypointNavigator::response_callback, this, std::placeholders::_1));
    }

    void response_callback(rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedFuture future) {
        try {
            auto result = future.get();
            RCLCPP_INFO(this->get_logger(), "Initial pose set successfully.");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set initial pose: %s", e.what());
        }
    }
    void load_waypoints(const std::string& file_path) {
        try {
            YAML::Node config = YAML::LoadFile(file_path);
            for (const auto& it : config["waypoints"]) {
                std::string name = it.first.as<std::string>();
                Waypoint wp;
                wp.x = it.second["x"].as<double>();
                wp.y = it.second["y"].as<double>();
                wp.yaw = it.second["yaw"].as<double>();
                waypoints_[name] = wp;
            }
            RCLCPP_INFO(this->get_logger(), "Loaded %lu waypoints.", waypoints_.size());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading YAML file: %s", e.what());
        }
    }

    void select_and_navigate() {
            std::cout << "\nAvailable waypoints:\n";
            for (const auto& wp : waypoints_) {
                std::cout << "  - " << wp.first<<":"<< wp.second.x << "," << wp.second.y << "\n";
            }

            std::string selected_wp;
            std::cout << "Enter waypoint name (or 'exit' to quit): ";
            std::cin >> selected_wp;

            if (selected_wp == "exit") {
                RCLCPP_INFO(this->get_logger(), "Exiting waypoint navigator.");
                return;
            }

            if (waypoints_.find(selected_wp) != waypoints_.end()) {
                send_goal(waypoints_[selected_wp]);
            } else {
                std::cout << "Invalid waypoint name. Try again.\n";
                select_and_navigate();
            }
    }

    void send_goal(const Waypoint& wp) {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = wp.x;
        goal_msg.pose.pose.position.y = wp.y;
        // goal_msg.pose.pose.orientation.z = sin(wp.yaw / 2);
        // goal_msg.pose.pose.orientation.w = cos(wp.yaw / 2);

        RCLCPP_INFO(this->get_logger(), "Navigating to x: %f, y: %f", wp.x, wp.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&WaypointNavigator::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&WaypointNavigator::goal_result_callback, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleNavigate::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected by server.");
            select_and_navigate();
        }
        RCLCPP_INFO(this->get_logger(), "Goal accepted, executing...");
    }

    void goal_result_callback(const GoalHandleNavigate::WrappedResult &result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Reached waypoint.");
            select_and_navigate();
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to reach waypoint.");
            select_and_navigate();

        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
