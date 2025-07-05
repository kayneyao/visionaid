from setuptools import find_packages
from setuptools import setup

setup(
    name='custom_nitros_message_filter_interfaces',
    version='3.2.5',
    packages=find_packages(
        include=('custom_nitros_message_filter_interfaces', 'custom_nitros_message_filter_interfaces.*')),
)
