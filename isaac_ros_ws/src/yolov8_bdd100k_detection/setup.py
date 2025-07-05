from setuptools import setup

package_name = 'yolov8_bdd100k_detection'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolov8_bdd100k_realsense.launch.py']),
        ('share/' + package_name + '/config', ['config/yolov8_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sophie',
    maintainer_email='sophie@visionaid.com',
    description='YOLOv8 BDD100K Object Detection with Isaac ROS and RealSense',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_processor = scripts.detection_processor:main',
        ],
    },
)
