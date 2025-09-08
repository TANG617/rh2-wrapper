from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rh2_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LiTang',
    maintainer_email='litang0617@outlook.com',
    description='ROS2 wrapper for RH2 robotic hand controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rh2_dual_hand_node = rh2_controller.rh2_node:main',
        ],
    },
    package_dir={'': '.'},  # 指定包的根目录
    python_requires='>=3.8',  # 指定Python版本要求
)