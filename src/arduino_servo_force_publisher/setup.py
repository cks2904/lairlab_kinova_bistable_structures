from setuptools import setup

package_name = 'arduino_servo_force_publisher'

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
    maintainer='yechan',
    maintainer_email='you@example.com',
    description='Arduino servo and force sensor ROS2 interface',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_force_control = arduino_servo_force_publisher.gripper_force_control:main',
            'servo_force_publisher = arduino_servo_force_publisher.servo_force_publisher:main',
        ],
    },
)

