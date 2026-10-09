from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'wr_person_following'

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
    description='Behavior-tree action and bounded local FSM for person following.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'person_follow_node = wr_person_following.person_follow_node:main',
        ],
    },
)
