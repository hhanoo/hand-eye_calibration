from setuptools import setup, find_packages
import os
from glob import glob

package_name = "hand_eye_calibration"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hhanoo",
    maintainer_email="woo980711@gmail.com",
    description="Multi-robot hand-eye calibration tool with PyQt5 GUI",
    license="MIT",
    entry_points={
        "console_scripts": [
            "gui_node = hand_eye_calibration.gui_node:main",
        ],
    },
)
