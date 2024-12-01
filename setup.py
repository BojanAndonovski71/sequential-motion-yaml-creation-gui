
from setuptools import setup

package_name = 'movement_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/movement_rviz.launch.py']),
        ('share/' + package_name + '/resource', [
            'resource/turtlebot_simple.urdf',
            'resource/rviz_config.rviz',
            'resource/test_movement.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bojan Andonovski',
    maintainer_email='bojan@example.com',
    description='Movement controller to load YAML and visualize movement in RViz',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = movement_controller.controller_node:main',
            'yaml_gui = movement_controller.yaml_gui:main'
        ],
    },
)
