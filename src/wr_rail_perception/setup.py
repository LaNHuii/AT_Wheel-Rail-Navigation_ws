"""wr_rail_perception ROS 2 Python 包的安装与资源声明。"""

from setuptools import find_packages, setup


package_name = 'wr_rail_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/rail_perception.yaml']),
        ('share/' + package_name + '/launch', ['launch/rail_perception.launch.py']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='AT Wheel-Rail Navigation Team',
    maintainer_email='maintainer@example.com',
    description='Geometric rail-entry perception from PointCloud2.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rail_entry_perception = wr_rail_perception.rail_entry_node:main',
        ],
    },
)
