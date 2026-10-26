"""wr_localization ROS 2 Python 包的安装与资源声明。"""

from setuptools import find_packages, setup

package_name = 'wr_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/local_ekf.yaml']),
        ('share/' + package_name + '/launch', ['launch/localization.launch.py']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='AT Wheel-Rail Navigation Team',
    maintainer_email='maintainer@example.com',
    description='Short-term wheel odometry and chassis-IMU fusion.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'wheel_odometry_node = wr_localization.wheel_odometry_node:main',
        ],
    },
)
