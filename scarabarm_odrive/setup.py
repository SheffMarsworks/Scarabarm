from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'scarabarm_odrive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # <-- This line installs launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='donggil',
    maintainer_email='dkim41@sheffield.ac.uk',
    description='Trajectory execution for ODrive using MoveIt trajectories',
    license='MIT',  # Or any valid SPDX license ID like 'Apache-2.0', 'BSD-3-Clause', etc.
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odrive_trajectory_executor = scarabarm_odrive.odrive_trajectory_executor:main',
        ],
    },
)
