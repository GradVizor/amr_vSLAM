from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("iris"),
            "config",
            "ros2_control.yaml",
        ]
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('iris'),
                'launch',
                'gazebo.launch.py',
            )
        )
    )

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
        arguments=["diff_controller", "-c", "/controller_manager"],
        remappings=[
            ("~/diff_controller/odom", "/odom"),
            ("diff_controller/cmd_vel", "/cmd_vel"),
        ])
    
    
    # Command to unpause Gazebo using `ros2 service call`
    unpause_gazebo = ExecuteProcess(
        cmd=["ros2", "service", "call", "/unpause_physics", "std_srvs/srv/Empty", "{}"],
        output="screen",
    )

    # joint_state_broadcaster --> diff_controller
    delay_diff_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[diff_controller],))
    
    return LaunchDescription([
        gazebo,
        unpause_gazebo,
        control_node,
        joint_state_broadcaster,
        delay_diff_controller_after_joint_state_broadcaster
    ])