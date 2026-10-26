"""wr_bringup ROS 2 Python 包安装声明。"""

from glob import glob
import os

from setuptools import setup


package_name = 'wr_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AT Wheel-Rail Navigation Team',
    maintainer_email='maintainer@example.com',
    description='Unified real-vehicle bringup.',
    license='Apache-2.0',
)
