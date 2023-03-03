from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
import launch_ros.actions

def generate_launch_description():

    version_arg = DeclareLaunchArgument(
        "version",
        default_value = TextSubstitution(text="2"),
        description = "version type [0, 1, 2]" ### 0: 2D, 1: 3D, 2: Dual
    )

    frequency_channel_arg = DeclareLaunchArgument(
        "frequency_level",
        default_value = TextSubstitution(text="0"),
        description = "frequency Ch. [0 to 15]"
    )

    pulse_control_arg = DeclareLaunchArgument(
        "pulse_control",
        default_value = TextSubstitution(text="0"),
        description = "pulse mode [0 (Auto), 1(Fixed)]" ### 0: Auto, 1: Manual
    )

    pulse_duration_arg = DeclareLaunchArgument(
        "pulse_duration",
        default_value = TextSubstitution(text="10000"),
        description = "pulse duration [0 to 10000] "
    )

    ld = LaunchDescription()

    lidar_node = launch_ros.actions.Node(
        package = 'cyglidar_d1',
        executable = 'CyglidarNode',
        output = 'screen',
        parameters=[
           {"port": "/dev/ttyUSB0"},
           {"baud_rate": 3000000},
           {"frame_id": "laser_frame"},
           {"fixed_frame": "/map"},
           {"run_mode": LaunchConfiguration("version")},
           {"frequency_channel": LaunchConfiguration("frequency_level")},
           {"duration_mode": LaunchConfiguration("pulse_control")},
           {"duration_value": LaunchConfiguration("pulse_duration")}
        ]
    )

    tf_node = launch_ros.actions.Node(
        package = 'tf2_ros', executable = "static_transform_publisher", name="to_laserframe",
        arguments = ["0", "0", "0", "0", "0", "0", "map", "laser_frame"]
    )

    ld.add_action(version_arg)
    ld.add_action(frequency_channel_arg)
    ld.add_action(pulse_control_arg)
    ld.add_action(pulse_duration_arg)
    ld.add_action(lidar_node)
    ld.add_action(tf_node)

    return ld
