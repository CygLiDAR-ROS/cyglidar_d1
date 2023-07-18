from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
import launch_ros.actions

def generate_launch_description():

    baudrate_change_arg = DeclareLaunchArgument(
        "select_baud_rate_number",
        default_value = TextSubstitution(text="1"),
        description = "version type [0, 1, 2]" ### 0: 3,000,000, 1: 115,200, 2: 57,600
    )

    ld = LaunchDescription()

    lidar_node = launch_ros.actions.Node(
        package = 'cyglidar_d1_ros2',
        executable = 'baudrate_changer',
        output = 'screen',
        parameters=[
           {"port": "/dev/ttyUSB0"},
           {"current_baud_rate": 3000000},
           {"select_baud_rate": LaunchConfiguration("select_baud_rate_number")}
        ]
    )

    ld.add_action(baudrate_change_arg)
    ld.add_action(lidar_node)

    return ld
