from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'rover_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rj',
    maintainer_email='rj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'lidar_processor_node = rover_controller.lidar_processor_node:main',
        'proximity_warning_node = rover_controller.proximity_warning_node:main',
        'emergency_stop_node = rover_controller.emergency_stop_node:main',
    ],
},
)
