from setuptools import find_packages, setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/control_system.launch.py']),
        ('share/' + package_name + '/launch', ['launch/px4_gazebo_iris.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lorenzo',
    maintainer_email='lorenzo@example.com',
    description='ROS2 control system for quadrotor drone',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_controller = drone_control.drone_controller:main',
            'state_estimator = drone_control.state_estimator:main',
            'trajectory_planner = drone_control.trajectory_planner:main',
            'px4_bridge = drone_control.px4_bridge:main',
            'mission_commander = drone_control.mission_commander:main',
        ],
    },
)
