#include "@(package_name)/@(package_name).hpp"
#include "pluginlib/class_list_macros.hpp"

namespace @(package_name)
{

void @(class_name)::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters)
{
    node_ = node;
    // RCLCPP_INFO(get_logger(), "@(class_name)::init");
}

void @(class_name)::step()
{
    // RCLCPP_INFO(get_logger(), "@(class_name)::step");
}

} // namespace @(package_name)

PLUGINLIB_EXPORT_CLASS(@(package_name)::@(class_name), webots_ros2_driver::PluginInterface)