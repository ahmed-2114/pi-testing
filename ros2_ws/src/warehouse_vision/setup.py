from setuptools import find_packages, setup

package_name = "warehouse_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/models", ["models/best.pt"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Audix Team",
    maintainer_email="audix@example.com",
    description="Camera and YOLO shelf scanning nodes for Audix.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hello_node = warehouse_vision.hello_node:main",
            "webcam_node = warehouse_vision.webcam_node:main",
            "vision_audit_node = warehouse_vision.vision_audit_node:main",
        ],
    },
)
