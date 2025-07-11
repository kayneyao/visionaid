from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2_benchmark',
    version='3.2.5',
    packages=find_packages(
        include=('ros2_benchmark', 'ros2_benchmark.*')),
)
