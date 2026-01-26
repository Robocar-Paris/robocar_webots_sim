from setuptools import find_packages, setup
import os

package_name = 'robocar_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'),
            ['resource/robocar.urdf']),
        (os.path.join('share', package_name, 'launch'),
            ['launch/robot_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Boris Cheng',
    maintainer_email='boris.cheng@example.com',
    description='Driver ROS 2 pour le Robocar dans Webots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = robocar_driver.test_node:main',
            'rl_trainer = robocar_driver.rl_environment:main',
            'keyboard_controller = robocar_driver.keyboard_controller:main',
            'webots_bridge = robocar_driver.webots_bridge:main',
        ],
    },
)
