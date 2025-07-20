from setuptools import setup

package_name = 'ee_serial_mux'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Serial bridge for end-effector commands',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ee_serial_mux = ee_serial_mux.ee_serial_mux:main',
        ],
    },
)
