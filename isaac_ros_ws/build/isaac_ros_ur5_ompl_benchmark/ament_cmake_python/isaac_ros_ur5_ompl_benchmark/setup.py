from setuptools import find_packages
from setuptools import setup

setup(
    name='isaac_ros_ur5_ompl_benchmark',
    version='3.2.6',
    packages=find_packages(
        include=('isaac_ros_ur5_ompl_benchmark', 'isaac_ros_ur5_ompl_benchmark.*')),
)
