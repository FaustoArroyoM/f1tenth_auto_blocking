from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'car_distance_estimator'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fausto',
    maintainer_email='fausto.arroyo.mantero@gmail.com',
    description='Multi-method car distance estimation using ZED-2 and YOLO<',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'car_distance_node = car_distance_estimator.car_distance_estimator_node:main',
            'test_distance_pub = car_distance_estimator.distance_publisher_node:main',
        ],
    },
)
