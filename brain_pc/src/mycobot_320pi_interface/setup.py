from setuptools import setup
import os
from glob import glob

package_name = "mycobot_320pi_interface"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*_launch.py")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=["setuptools", "pyzmq"],
    extras_require={
        "test": ["pytest"],
    },
    zip_safe=True,
    maintainer="Usama",
    maintainer_email="nrih.robotics@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "mycobot_320pi_interface = mycobot_320pi_interface.mycobot_320pi_interface:main"
        ],
    },
)
