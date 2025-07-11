from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2_benchmark_interfaces',
    version='3.2.5',
    packages=find_packages(
        include=('ros2_benchmark_interfaces', 'ros2_benchmark_interfaces.*')),
)
