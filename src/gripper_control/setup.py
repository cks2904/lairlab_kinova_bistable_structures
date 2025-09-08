from setuptools import setup
package_name = 'gripper_control'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    maintainer='Yechan',
    maintainer_email='yechan@example.com',
    description='Gripper control node for Robotiq 2F-85',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_action_client = gripper_control.gripper_action_client:main',
            'gripper_force_monitor = gripper_control.gripper_force_monitor:main',
            'gripper_force_proxy = gripper_control.gripper_force_proxy:main',
        ],
    },
)
