import os
from glob import glob

from setuptools import find_packages, setup

package_name = "auv_2d_missions"

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
    maintainer="aatmaj",
    maintainer_email="na22b018@smail.iitm.ac.in",
    description="Mission-level guidance nodes for 2D AUV autonomy.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "point_tracking_mission_2d = auv_2d_missions.point_tracking_mission:main",
        ],
    },
)

