from setuptools import setup

package_name = 'odrive_traj_exec'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/start_odrive_and_arm.launch.py']),
        ('share/' + package_name + '/launch', ['launch/turn_off_all.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your name',
    maintainer_email='you@example.com',
    description='ODrive trajectory executor',
    license='Apache License 2.0',
    tests_require=['pytest'],
    eentry_points={
        'console_scripts': [
            'arm_axes = odrive_traj_exec.arm_axes:main',
        ],
    },
)
