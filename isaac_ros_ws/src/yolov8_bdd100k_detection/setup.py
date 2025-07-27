from setuptools import setup

package_name = 'yolov8_bdd100k_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolov8_bdd100k_realsense.launch.py']),
        ('share/' + package_name + '/config', ['config/camera_params.yaml']),
        ('share/' + package_name + '/config', ['config/detection_params.yaml']),
        ('share/' + package_name + '/config', ['config/yolov8_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sophie',
    maintainer_email='your_email@example.com',
    description='Taiwan 17-class traffic safety detection system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov8_camera_node = yolov8_bdd100k_detection.yolov8_camera_node:main',
            'detection_processor = yolov8_bdd100k_detection.detection_processor:main',
        ],
    },
)
