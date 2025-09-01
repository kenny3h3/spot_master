from setuptools import setup
import os
from glob import glob

package_name = 'bno085_imu_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'adafruit-circuitpython-bno08x',
        'adafruit-extended-bus'  # Korrigierter Paketname
    ],
    zip_safe=True,
    maintainer='winston',
    maintainer_email='winston@example.com',
    description='BNO085 IMU Python Node',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'bno085_imu_node = bno085_imu_py.node:main',
        ],
    },
)