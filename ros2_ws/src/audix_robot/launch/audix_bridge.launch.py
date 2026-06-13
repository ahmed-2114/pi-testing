from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    mock_ir = LaunchConfiguration("mock_ir")
    mock_gpio = LaunchConfiguration("mock_gpio")
    ir_enabled = LaunchConfiguration("ir_enabled")
    ir_logic = LaunchConfiguration("ir_logic")
    init_imu_on_start = LaunchConfiguration("init_imu_on_start")
    reset_odom_on_start = LaunchConfiguration("reset_odom_on_start")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="audix"),
            DeclareLaunchArgument("port", default_value="/dev/ttyAMA0"),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("mock_ir", default_value="false"),
            DeclareLaunchArgument("mock_gpio", default_value="false"),
            DeclareLaunchArgument("ir_enabled", default_value="true"),
            DeclareLaunchArgument("ir_logic", default_value="baseline"),
            DeclareLaunchArgument("init_imu_on_start", default_value="true"),
            DeclareLaunchArgument("reset_odom_on_start", default_value="true"),
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                name="micro_ros_agent",
                output="screen",
                arguments=["serial", "--dev", port, "--baudrate", baud],
            ),
            Node(
                package="audix_robot",
                executable="micro_ros_base_node",
                name="micro_ros_base",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "init_imu_on_start": ParameterValue(init_imu_on_start, value_type=bool),
                        "reset_odom_on_start": ParameterValue(reset_odom_on_start, value_type=bool),
                    }
                ],
            ),
            Node(
                package="audix_robot",
                executable="gpio_hardware_node",
                name="gpio_hardware",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "mock_gpio": ParameterValue(mock_gpio, value_type=bool),
                        "mock_ir": ParameterValue(mock_ir, value_type=bool),
                        "ir_enabled": ParameterValue(ir_enabled, value_type=bool),
                        "ir_logic": ir_logic,
                    }
                ],
            ),
        ]
    )
