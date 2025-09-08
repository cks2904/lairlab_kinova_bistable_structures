from setuptools import setup

package_name = 'ros2_arduino_servo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='예찬',
    maintainer_email='your_email@example.com',
    description='ROS2 package to control Arduino servo via serial',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'servo_node = ros2_arduino_servo.servo_node:main',
        ],
    },
)

