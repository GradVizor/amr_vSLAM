import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = True
    # use_sim_time = LaunchConfiguration('use_sim_time')
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('perceptor'))
    xacro_file = os.path.join(pkg_path,'description','urdf','robot.urdf')
    robot_description_config = xacro.process_file(xacro_file)
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[params]
    )
    # Launch!
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher
    ])