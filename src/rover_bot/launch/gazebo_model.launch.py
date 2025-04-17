from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
import xacro 

def generate_launch_description():
    # Paths
    
    
    
    
    robotXacroName ='differential_drive_robot'
    namePackage ='rover_bot'
    modelFileRelativePath = 'model/robot.xacro'
    worldFileRelativePath = 'model/empty_world.world'
    pathModelFile = os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage),worldFileRelativePath)
    

    rviz_config_file = os.path.join(
        get_package_share_directory(namePackage),
        'rviz',
        'rover_bot.rviz'  
    )




    robotDescription = xacro.process_file(pathModelFile).toxml()
    
    
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'))
    
    
    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch,launch_arguments={'world': pathWorldFile}.items())
    
    


    rvizNode = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]  
        )
    

    # Spawn robot entity in Gazebo
    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', robotXacroName,
            '-x', '0',
            '-y', '0',
            '-z', '0.01'
        ],
        output='screen'
    )

    # Robot state publisher
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True
        }]
    )

    Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),


    # Launch description
    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(rvizNode)

    return launchDescriptionObject

