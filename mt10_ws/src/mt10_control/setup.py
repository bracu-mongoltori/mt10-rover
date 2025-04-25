from setuptools import find_packages, setup
import os
from glob import glob

package_name = "mt10_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mahir",
    maintainer_email="mahir@todo.todo",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "diff_drive = mt10_control.diff_drive:main",
            "autonomous = mt10_control.autonomous_v1:main",
            "dummy = mt10_control.dummy_sbg:main",
            "gps_txt = mt10_control.test:main",
            "aruco_new = mt10_control.ar_with_new_logic:main",
            "aruco_test = mt10_control.ar_with_new_logic_test:main",
            "mallet = mt10_control.mallet_detect:main",
            "bottle = mt10_control.bottle_detect:main",
            "point_follower = mt10_control.point_follow:main",
            "zed_multi = mt10_control.zed_multicamera:main",
            "zed = mt10_control.zed_camera:main",
            "overlap = mt10_control.overlap:main",
            "pcl_view = mt10_control.point_cloud_view:main",
            "angle = mt10_control.sbg_angle:main",
            "port_finder = mt10_control.port_finder:main",
            "auto_witmotion= mt10_control.autonomous_witmotion:main",
            "telemetry_read = mt10_control.p900_write:main",
        ],
    },
)
