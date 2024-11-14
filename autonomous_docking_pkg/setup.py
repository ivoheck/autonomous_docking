#https://roboticsbackend.com/ros2-run-and-ros2-node-start-debug-ros2-nodes/
#https://roboticsbackend.com/ros2-launch-file-example/

from setuptools import find_packages, setup

package_name = 'autonomous_docking_pkg'

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
            'launch/autonomous_docking_launch.py',
            'launch/test_launch.py', 
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivo',
    maintainer_email='ivo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'my_node = {package_name}.test:main',
            f'my_node2 = {package_name}.test2:main',
            f'my_node3 = {package_name}.test3:main',
            f'motor_controller_node = {package_name}.motor_controller:main',
            f'drive_to_qr_node = {package_name}.drive_to_qr:main',
            f'scan_drive_to_qr_node = {package_name}.scan_drive_to_qr:main',
            f'face_wall_node = {package_name}.face_wall:main',
            f'drive_to_wall_node = {package_name}.drive_to_wall:main',
            f'qr_scan_node = {package_name}.qr_scan:main',
            f'qr_center_node = {package_name}.qr_center:main',
            f'scan_find_qr_node = {package_name}.scan_find_qr:main',
            f'find_qr_node = {package_name}.find_qr:main',
            f'probe_node = {package_name}.probe:main',
            f'probe_docking_node = {package_name}.probe_docking:main',
            f'probe_docking_service_node = {package_name}.probe_docking_service:main',
            f'odometry_publisher_node = {package_name}.odometry:main',
            f'lidar_fillter_node = {package_name}.lidar_fillter:main',
            f'lock_controller_node = {package_name}.lock_controller:main',
            f'undock_node = {package_name}.undock:main',
            f'start_docking_node = {package_name}.start_docking:main',
            f'undocking_node = {package_name}.undocking:main',
        ],
    },
)
