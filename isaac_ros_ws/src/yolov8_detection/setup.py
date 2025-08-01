from setuptools import setup

package_name = 'yolov8_detection'  # UPDATED

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # This must match the actual directory name
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolov8_realsense.launch.py']),
        ('share/' + package_name + '/config', ['config/camera_params.yaml']),
        ('share/' + package_name + '/config', ['config/detection_params.yaml']),
        ('share/' + package_name + '/config', ['config/yolov8_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sophie',
    maintainer_email='sophiehsuu@gmail.com',
    description='Taiwan 11-class traffic safety detection system',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'yolov8_camera_node = yolov8_detection.yolov8_camera_node:main',
            'detection_processor = yolov8_detection.detection_processor:main',
        ],
    },
)
