from setuptools import setup
from glob import glob

package_name = 'motion_cmd'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'README.md']),
    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ('share/' + package_name + '/params', glob('motion_cmd/params/*.yaml')),
]

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, package_name + '.gaits', package_name + '.utils', package_name + '.params'],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='You',
    maintainer_email='you@example.com',
    description='ROS 2 Jazzy motion commander for Spot-Micro/SMOV; dual PCA9685; optional BNO085 IMU; independent of walking_gait.',
    license='MIT',
    entry_points={'console_scripts': ['motion_cmd_node = motion_cmd.motion_cmd_node:main']},
)
