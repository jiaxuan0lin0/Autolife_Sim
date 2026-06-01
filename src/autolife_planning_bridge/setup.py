from glob import glob

from setuptools import find_packages, setup

package_name = "autolife_planning_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["numpy", "setuptools"],
    zip_safe=True,
    maintainer="JiaxuanLin",
    maintainer_email="jiaxuan0lin0@gmail.com",
    description="ROS 2 bridge from Autolife planning actions to the existing controller stack.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "planning_server = autolife_planning_bridge.planning_server:main",
        ],
    },
)
