#pragma once

#include <rclcpp/rclcpp.hpp>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots/Robot.hpp>
// #include <webots/Motor.hpp>

namespace @(package_name)
{

class @(class_name) : public webots_ros2_driver::PluginInterface {
public:
    @(class_name)() { }

    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
    void step() override;

private:
    rclcpp::Logger get_logger()
    {
        return node_->get_logger();
    }
    
    webots_ros2_driver::WebotsNode *node_;
    
};

} // namespace @(package_name)