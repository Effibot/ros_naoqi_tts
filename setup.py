import os
from glob import glob

from setuptools import setup

package_name = "ros_naoqi_tts"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "script"),
            glob(os.path.join("script", "*.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="andrea efficace",
    maintainer_email="andrea.efficace1@gmail.com",
    description="implementation of tts service through android, naoqi sdk and ros2",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tts_node = ros_naoqi_tts.tts_node:main",
            "web_server = ros_naoqi_tts.web_server:main",
        ],
    },
)
