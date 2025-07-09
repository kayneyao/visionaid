# vlm_scene_understanding/setup.py
from setuptools import setup

package_name = 'vlm_scene_understanding'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sophie',
    maintainer_email='sophiehsuu@gmail.com',
    description='BLIP-2 VLM for scene understanding in assistive navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blip2_node = vlm_scene_understanding.blip2_node:main',
        ],
    },
)
