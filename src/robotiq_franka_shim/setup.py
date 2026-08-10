from glob import glob

from setuptools import setup

package_name = "robotiq_franka_shim"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/urdf", glob("urdf/*.xacro")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SONG Yeke",
    maintainer_email="songykee@gmail.com",
    description="franka_gripper-compatible shim for a Robotiq 2F-85 on an FR3.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "gripper_shim = robotiq_franka_shim.gripper_shim:main",
        ],
    },
)
