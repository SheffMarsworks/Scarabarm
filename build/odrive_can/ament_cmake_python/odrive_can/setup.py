from setuptools import find_packages
from setuptools import setup

setup(
    name='odrive_can',
    version='0.0.1',
    packages=find_packages(
        include=('odrive_can', 'odrive_can.*')),
)
