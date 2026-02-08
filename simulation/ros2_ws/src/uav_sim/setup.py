"""Setup for the uav_sim ROS 2 package."""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "uav_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament resource index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # package manifest
        ("share/" + package_name, ["package.xml"]),
        # launch files
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        # config files
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ratan Lal Bunkar",
    maintainer_email="ratan@todo.todo",
    description="ROS 2 PID controller node and SITL launch files for UAV simulation",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pid_controller_node = uav_sim.pid_controller_node:main",
            "rl_policy_node = uav_sim.rl_policy_node:main",
        ],
    },
)
