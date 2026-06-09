from setuptools import setup

package_name = "sevenbot_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Erik de Bruijn",
    maintainer_email="erik@stekker.app",
    description="ROS2 bridge voor 7Bot robotarm via USB serial.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "sevenbot_bridge = sevenbot_bridge.bridge_node:main",
        ],
    },
)
