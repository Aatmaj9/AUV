import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'auv_2d_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aatmaj',
    maintainer_email='na22b018@smail.iitm.ac.in',
    description='Common planar AUV controller for 2D missions',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller_2d = auv_2d_control.heading_speed_controller:main',
            'thruster_allocator_2d = auv_2d_control.thruster_allocator_2d:main',
        ],
    },
)
