from setuptools import find_packages, setup

package_name = 'auto_dock_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/auto_dock_bringup_launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivo',
    maintainer_email='ivo@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'find_qr_node = {package_name}.find_qr:main',
            f'drive_to_qr_node = {package_name}.drive_to_qr:main',
            f'qr_scan_node = {package_name}.qr_scan:main',
            f'face_wall_node = {package_name}.face_wall:main',
            f'drive_to_wall_node = {package_name}.drive_to_wall:main',
            f'qr_center_node = {package_name}.qr_center:main',
            f'final_docking_node = {package_name}.final_docking:main',
            f'docking_manager_node = {package_name}.docking_manager:main',
            f'undocking_node = {package_name}.undocking:main',
        ],
    },
)
