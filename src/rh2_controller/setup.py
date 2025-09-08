from setuptools import setup, find_packages

package_name = 'rh2_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rh2_controller.launch.py']),
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
            'rh2_controller_node = rh2_controller.rh2_ros_wrapper:main',
            'rh2_dual_hand_node = rh2_controller.rh2_node:main',
            'rh2_test_client = rh2_controller.test_client:main',
        ],
    },
)
