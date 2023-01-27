[![build](https://github.com/Kotakku/ros2webotscli/actions/workflows/build.yaml/badge.svg)](https://github.com/Kotakku/ros2webotscli/actions/workflows/build.yaml)

# ros2webotscli
ROS 2 command line interface tools for ROS 2 + Webots

## Usage

Run `ros2 webots pkg_create <package name> <class name>` for create package template for ROS 2 + Webots project.

Run `ros2 webots run` for run webots.

Run `ros2 webots --help` for more information on individual command usage.

### Example

`ros2 webots pkg_create my_robot_driver MyRobotDriver --dependencies geometry_msgs`
