from setuptools import setup
import os
from glob import glob

package_name = 'yolov8_bdd100k_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sophie',
    maintainer_email='sophie@example.com',
    description='YOLOv8 BDD100K detection package for Isaac ROS',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'yolov8_camera_node = yolov8_bdd100k_detection.yolov8_camera_node:main',
        'detection_processor = yolov8_bdd100k_detection.detection_processor:main',
    ],
},

)
