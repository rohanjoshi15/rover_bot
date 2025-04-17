from setuptools import find_packages, setup

package_name = 'rover_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
	data_files=[
	    ('share/ament_index/resource_index/packages',
		['resource/' + package_name]),
	    ('share/' + package_name, ['package.xml']),
	    (f'share/{package_name}/launch', ['launch/gazebo_model.launch.py']),
	    (f'share/{package_name}/model', ['model/robot.xacro']),
	    (f'share/{package_name}/model', ['model/robot.xacro', 'model/robot.gazebo']),
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
        	# "node1 = rover_bot.lidar_processor_node:main" ,
        	# "node2 = rover_bot.proximity_warning_node:main",
        	# "node3 = rover_bot.emergency_stop_node:main"
        ],
    },
)
