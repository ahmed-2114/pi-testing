from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    dashboard_port = LaunchConfiguration("dashboard_port")
    allow_placeholder_audit = LaunchConfiguration("allow_placeholder_audit")
    camera_enabled = LaunchConfiguration("camera_enabled")
    camera_index = LaunchConfiguration("camera_index")
    vision_enabled = LaunchConfiguration("vision_enabled")
    vision_confidence = LaunchConfiguration("vision_confidence")
    vision_target_count = LaunchConfiguration("vision_target_count")
    vision_scan_settle = LaunchConfiguration("vision_scan_settle")
    audit_side_1_level_1_shelf_id = LaunchConfiguration("audit_side_1_level_1_shelf_id")
    audit_side_1_level_2_shelf_id = LaunchConfiguration("audit_side_1_level_2_shelf_id")
    audit_side_2_level_1_shelf_id = LaunchConfiguration("audit_side_2_level_1_shelf_id")
    audit_side_2_level_2_shelf_id = LaunchConfiguration("audit_side_2_level_2_shelf_id")

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
            DeclareLaunchArgument("dashboard_port", default_value="8080"),
            DeclareLaunchArgument("allow_placeholder_audit", default_value="false"),
            DeclareLaunchArgument("camera_enabled", default_value="true"),
            DeclareLaunchArgument("camera_index", default_value="0"),
            DeclareLaunchArgument("vision_enabled", default_value="true"),
            DeclareLaunchArgument("vision_confidence", default_value="0.5"),
            DeclareLaunchArgument("vision_target_count", default_value="2"),
            DeclareLaunchArgument("vision_scan_settle", default_value="0.5"),
            DeclareLaunchArgument("audit_side_1_level_1_shelf_id", default_value="beans_can"),
            DeclareLaunchArgument("audit_side_1_level_2_shelf_id", default_value="indomie"),
            DeclareLaunchArgument("audit_side_2_level_1_shelf_id", default_value="indomie"),
            DeclareLaunchArgument("audit_side_2_level_2_shelf_id", default_value="fruit_rings_cereal"),
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
            Node(
                package="audix_robot",
                executable="robot_manager_node",
                name="robot_manager",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "allow_placeholder_audit": ParameterValue(allow_placeholder_audit, value_type=bool),
                        "vision_scan_settle_s": ParameterValue(vision_scan_settle, value_type=float),
                        "audit_side_1_level_1_shelf_id": audit_side_1_level_1_shelf_id,
                        "audit_side_1_level_2_shelf_id": audit_side_1_level_2_shelf_id,
                        "audit_side_2_level_1_shelf_id": audit_side_2_level_1_shelf_id,
                        "audit_side_2_level_2_shelf_id": audit_side_2_level_2_shelf_id,
                    }
                ],
            ),
            Node(
                package="warehouse_vision",
                executable="webcam_node",
                name="webcam",
                namespace=namespace,
                output="screen",
                condition=IfCondition(camera_enabled),
                parameters=[
                    {
                        "camera_index": ParameterValue(camera_index, value_type=int),
                        "image_topic": "image_raw",
                    }
                ],
            ),
            Node(
                package="warehouse_vision",
                executable="vision_audit_node",
                name="vision_audit",
                namespace=namespace,
                output="screen",
                condition=IfCondition(vision_enabled),
                parameters=[
                    {
                        "image_topic": "image_raw",
                        "scan_service": "scan_shelf",
                        "confidence_threshold": ParameterValue(vision_confidence, value_type=float),
                        "target_count": ParameterValue(vision_target_count, value_type=int),
                    }
                ],
            ),
            Node(
                package="audix_robot",
                executable="web_dashboard_node",
                name="web_dashboard",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "host": "0.0.0.0",
                        "port": ParameterValue(dashboard_port, value_type=int),
                    }
                ],
            ),
        ]
    )
