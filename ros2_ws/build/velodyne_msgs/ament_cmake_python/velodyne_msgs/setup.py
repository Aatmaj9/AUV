from setuptools import find_packages
from setuptools import setup

setup(
    name='velodyne_msgs',
    version='2.5.1',
    packages=find_packages(
        include=('velodyne_msgs', 'velodyne_msgs.*')),
)
