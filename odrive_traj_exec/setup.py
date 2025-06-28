from setuptools import setup
package_name = 'odrive_traj_exec'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'traj_exec = odrive_traj_exec.traj_node:main',
        ],
    },
)
