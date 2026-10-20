from glob import glob

from setuptools import find_packages, setup

package_name = 'pico_lsm6ds3_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathan Lewis',
    maintainer_email='git@nrlewis.dev',
    description='Driver and orientation fusion for the Pico + LSM6DS3 USB HID IMU',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = pico_lsm6ds3_ros2.driver_node:main',
            'measure_noise = pico_lsm6ds3_ros2.measure_noise:main',
        ],
    },
)
