import glob

from setuptools import find_packages, setup

package_name = "franka_robotiq_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch.py")),
        ("share/" + package_name + "/urdf", glob.glob("urdf/*.urdf.xacro")),
    ],
    include_package_data=True,
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="SONG Yeke",
    maintainer_email="songykee@gmail.com",
    description="Robotiq 2F-85 bringup for the GELLO -> FR3 teleoperation stack.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robotiq_gripper_client = "
            "franka_robotiq_bringup.robotiq_gripper_client:main",
        ],
    },
)
