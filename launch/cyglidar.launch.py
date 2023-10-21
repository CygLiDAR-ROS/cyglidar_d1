from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    baud_rate_value_arg = DeclareLaunchArgument(
        "baud_rate_mode",
        default_value = TextSubstitution(text="0"),
        description = "baud rate value [0:(3,000,000), 1:(921,600), 2:(115,200), 3:(57,600)]"
    )

    version_arg = DeclareLaunchArgument(
        "version",
        default_value = TextSubstitution(text="2"),
        description = "version type [0:(MODE_2D), 1:(MODE_3D), 2:(MODE_DUAL)]"
    )

    frequency_channel_arg = DeclareLaunchArgument(
        "frequency_level",
        default_value = TextSubstitution(text="0"),
        description = "frequency Ch. [0 to 15]"
    )

    pulse_control_arg = DeclareLaunchArgument(
        "pulse_control",
        default_value = TextSubstitution(text="0"),
        description = "pulse mode [0 : (Auto), 1 : (Manual)]"
    )

    pulse_duration_arg = DeclareLaunchArgument(
        "pulse_duration",
        default_value = TextSubstitution(text="10000"),
        description = "pulse duration [0 to 10000] "
    )

    color_display_arg = DeclareLaunchArgument(
        "color_display",
        default_value = TextSubstitution(text="0"),
        description = "color mode [0 (MODE_HUE), 1 (MODE_RGB), 2 (MODE_GRAY)]"
    )

    min_resolution_arg = DeclareLaunchArgument(
        "min_resolution_value",
        default_value = TextSubstitution(text="0"),
        description = "min_resolution [0 to 3000] "
    )

    max_resolution_arg = DeclareLaunchArgument(
        "max_resolution_value",
        default_value = TextSubstitution(text="4000"),
        description = "max_resolution [0 to 3000] "
    )

    ld = LaunchDescription()

    lidar_node = launch_ros.actions.Node(
        package = 'cyglidar_d1_ros2',
        executable = 'cyglidar_d1_publisher',
        output = 'screen',
        parameters=[
           {"port": "/dev/ttyUSB0"},
           {"baud_rate": LaunchConfiguration("baud_rate_mode")},
           {"frame_id": "laser_frame"},
           {"fixed_frame": "/map"},
           {"run_mode": LaunchConfiguration("version")},
           {"frequency_channel": LaunchConfiguration("frequency_level")},
           {"duration_mode": LaunchConfiguration("pulse_control")},
           {"duration_value": LaunchConfiguration("pulse_duration")},
           {"color_mode": LaunchConfiguration("color_display")},
           {"min_resolution": LaunchConfiguration("min_resolution_value")},
           {"max_resolution": LaunchConfiguration("max_resolution_value")}
        ]
    )

    tf_node = launch_ros.actions.Node(
        package = 'tf2_ros', executable = "static_transform_publisher", name="to_laserframe",
        arguments = ["0", "0", "0", "0", "0", "0", "map", "laser_frame"]
    )

    ld.add_action(baud_rate_value_arg)
    ld.add_action(version_arg)
    ld.add_action(frequency_channel_arg)
    ld.add_action(pulse_control_arg)
    ld.add_action(pulse_duration_arg)
    ld.add_action(color_display_arg)
    ld.add_action(min_resolution_arg)
    ld.add_action(max_resolution_arg)
    ld.add_action(lidar_node)
    ld.add_action(tf_node)

    return ld
