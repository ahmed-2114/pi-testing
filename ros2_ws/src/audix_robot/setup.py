from setuptools import setup

package_name = "audix_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            ["launch/audix_bridge.launch.py", "launch/audix_main.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Audix Team",
    maintainer_email="audix@example.com",
    description="ROS 2 bridge nodes for the Audix robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "micro_ros_base_node = audix_robot.micro_ros_base_node:main",
            "gpio_hardware_node = audix_robot.gpio_hardware_node:main",
            "robot_manager_node = audix_robot.robot_manager_node:main",
            "web_dashboard_node = audix_robot.web_dashboard_node:main",
            "terminal_move_node = audix_robot.terminal_move_node:main",
            "terminal_teleop_node = audix_robot.terminal_teleop_node:main",
        ],
    },
)
