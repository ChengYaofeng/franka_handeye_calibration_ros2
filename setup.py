from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'franka_handeye_calibration_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cyf',
    maintainer_email='yaofeng@liverpool.ac.uk',
    description='fr3_handeye_calibration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_aruco_publisher = franka_handeye_calibration_ros2.calibration_aruco_publisher:main',
            'follow_aruco_marker = franka_handeye_calibration_ros2.follow_aruco_marker:main',
            'handeye_publisher = franka_handeye_calibration_ros2.handeye_publisher:main',
        ],
    },
)
