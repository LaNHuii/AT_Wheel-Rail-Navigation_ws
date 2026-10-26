"""wr_runtime_monitor ROS 2 Python 包安装声明。"""

from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'wr_runtime_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='AT Wheel-Rail Navigation Team',
    maintainer_email='maintainer@example.com',
    description='Sensor health and autonomy-readiness monitor.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'runtime_monitor_node = wr_runtime_monitor.runtime_monitor_node:main',
        ],
    },
)
