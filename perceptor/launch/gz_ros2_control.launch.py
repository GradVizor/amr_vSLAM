import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # =========== Launch Arguments ============
    
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    share_dir = get_package_share_directory('perceptor')

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("perceptor"),
            "config",
            "ros2_control.yaml",])

    bridge_params = os.path.join(
        share_dir,
        'config',
        'bridge_params.yaml')


    # ============= Launch Nodes ==============
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                share_dir,
                'launch',
                'gz_sim.launch.py',)))

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[("~/robot_description", "/robot_description"),],
        output="both",)
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", 
                   "--controller-manager", 
                   "/controller_manager"])
    
    diff_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "-c", "/controller_manager"])
    
    ros_gz_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',],
        output='screen',)

    delay_diff_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[diff_controller],))
        
    sim_true = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock')
    
    return LaunchDescription([
        gazebo,
        sim_true,
        ros_gz_bridge,
        control_node,
        joint_state_broadcaster,
        delay_diff_controller_after_joint_state_broadcaster
    ])