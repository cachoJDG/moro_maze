from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'moro_maze'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))), # Register all launchfiles
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))), # Register all map files
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))), # Register all gazebo worlds
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))), # Register nav2 params
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))), # Register rviz configs
    ],
    install_requires=['setuptools', 'scikit-learn'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localisation_node = moro_maze.localisation_node:main',
            'global_planner_node = moro_maze.global_planner_node:main',
            'local_controller_node = moro_maze.local_controller_node:main',
        ],
    },
)
