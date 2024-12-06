from setuptools import find_packages, setup

package_name = 'robot_controll_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/bringup_launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivo',
    maintainer_email='ivoheck99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'odometry_publisher_node = {package_name}.odometry:main',
            f'motor_controller_node = {package_name}.motor_controller:main',
            f'lock_controller_node = {package_name}.lock_controller:main',
            f'lidar_filter_node = {package_name}.lidar_filter:main',
        ],
    },
)
