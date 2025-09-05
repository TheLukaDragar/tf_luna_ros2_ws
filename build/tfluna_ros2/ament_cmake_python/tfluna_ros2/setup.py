from setuptools import find_packages
from setuptools import setup

setup(
    name='tfluna_ros2',
    version='1.0.0',
    packages=find_packages(
        include=('tfluna_ros2', 'tfluna_ros2.*')),
)
