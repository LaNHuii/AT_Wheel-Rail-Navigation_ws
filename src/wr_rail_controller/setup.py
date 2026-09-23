from setuptools import find_packages, setup


package_name = 'wr_rail_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/rail_controller.yaml',
            'config/rail_controller_sim.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='AT Wheel-Rail Navigation Team',
    maintainer_email='maintainer@example.com',
    description='Replaceable rail clamp and rail-mode simulation controller.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rail_controller_node = wr_rail_controller.rail_controller_node:main',
        ],
    },
)
