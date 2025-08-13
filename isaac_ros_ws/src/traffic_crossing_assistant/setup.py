from setuptools import setup
import os
from glob import glob

package_name = 'traffic_crossing_assistant'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sophie',
    maintainer_email='your_email@example.com',
    description='Taiwan Traffic Safety System with 3-Priority Hierarchy',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            # Core system components
            'vehicle_movement_analyzer = traffic_crossing_assistant.vehicle_movement_analyzer:main',
            'crosswalk_analyzer = traffic_crossing_assistant.crosswalk_analyzer:main',
            'traffic_light_analyzer = traffic_crossing_assistant.traffic_light_analyzer:main',
            'decision_engine = traffic_crossing_assistant.decision_engine:main',
            'ego_motion_compensator = traffic_crossing_assistant.ego_motion_compensator:main',
            'simple_3d_analyzer = traffic_crossing_assistant.simple_3d_analyzer:main',
            
            # Audio and feedback systems
            'audio_feedback_system = traffic_crossing_assistant.audio_feedback_system:main',
            'enhanced_audio_system = traffic_crossing_assistant.enhanced_audio_system:main',
            
            # Multimodal safety coordinator
            'multimodal_safety_coordinator = traffic_crossing_assistant.multimodal_safety_coordinator:main',
            
            # Testing and validation scripts
            'test_complete_system = scripts.test_complete_system:main',
            'test_vlm_integration = scripts.test_vlm_integration:main',
            'test_enhanced_3d_system = scripts.test_enhanced_3d_system:main',
            'validate_system = scripts.validate_system:main',

            # Experiment utilities
            'experiment_recorder = scripts.experiment_recorder:main',
            'analyze_experiments = scripts.analyze_experiments:main',
        ],
    },
)
