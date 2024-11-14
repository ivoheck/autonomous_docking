from setuptools import find_packages, setup

package_name = 'global_manager_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/global_planer_launch.py',
            'launch/global_navigation_launch.py', 
        ]),
        ('share/' + package_name + '/config', ['config/workplace.yaml']),
        
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
            f'global_planer_node = {package_name}.global_planer:main',
            f'nav2_start_node = {package_name}.nav2_start:main',
            f'path_follower_node = {package_name}.path_follower:main'
        ],
    },
)
