from setuptools import setup, find_packages

package_name = 'scarabarm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Scarab arm CAN-bridge controllers (Python)',
    license='Apache-2.0',
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/bringup_pi.launch.py']),
    ],
    entry_points={
        'console_scripts': [
            'scarab_controller = scarabarm_control.scarabarm_controller_node:main',
            'scarab_js_agg   = scarabarm_control.joint_state_aggregator:main',
        ],
    },
)
