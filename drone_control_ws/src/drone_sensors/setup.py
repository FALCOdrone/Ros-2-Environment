from setuptools import find_packages, setup

package_name = 'drone_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lorenzo',
    maintainer_email='lorenzo@example.com',
    description='Sensor simulation nodes for quadrotor drone',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'barometer_sim = drone_sensors.barometer_sim:main',
            'magnetometer_sim = drone_sensors.magnetometer_sim:main',
        ],
    },
)
