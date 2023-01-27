import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix

def generate_launch_description():
    # params
    package_name = '@(package_name)'
    urdf_file = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    wdt_file = os.path.join(package_dir, 'worlds', 'my_world.wbt')
    robot_name = 'my_robot'

    package_dir = get_package_share_directory(package_name)
    robot_description = pathlib.Path(urdf_file).read_text()

    webots = WebotsLauncher(
        world=wdt_file
    )

    ros2_supervisor = Ros2SupervisorLauncher()

    robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + robot_name},
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    return LaunchDescription([
        webots,
        robot_driver,
        ros2_supervisor,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])