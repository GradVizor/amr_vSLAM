from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from os.path import join
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration,PythonExpression


def generate_launch_description():
    
    # =========== Launch Arguments ============    
    share_dir = get_package_share_directory('iris_description')
    bringup_dir = get_package_share_directory('iris_bringup')

    xacro_file = os.path.join(share_dir, 'urdf', 'robot.urdf')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    world_file = os.path.join(share_dir, "worlds", "small_warehouse.sdf")
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    # bridge_params = os.path.join(
    #     bringup_dir,
    #     'config',
    #     'gz_bridge.yaml')
    # world = [os.path.join(share_dir, 'worlds', 'med_object_world.sdf')] #


    # ============= Launch Nodes ==============

    worlds_resource = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=join(share_dir, "worlds"))

    models_resource = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=join(share_dir, "models"))
        
    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir,
                    'launch',
                    'rsp.launch.py',)))

    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
    #     launch_arguments={
    #         "gz_args" : PythonExpression(["'", world_file, " -r'"])

    #     }.items()
    # )
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression(["'", world_file, " -r'"])

        }.items()
    )
    
    iris_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_urdf,
                   '-name', 'iris',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.15',
                   '-allow_renaming', 'true'],)
    
    # ros_gz_bridge = Node(
    #     package='ros_ign_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         '--ros-args',
    #         '-p',
    #         f'config_file:={bridge_params}',],
    #     output='screen',)
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            # "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            # "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/stereo_camera/left/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "stereo_camera/right/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            # "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/world/default/model/iris_bringup/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"
        ],
        remappings=[
            ('/world/default/model/iris_bringup/joint_state', 'joint_states'),
            # ('/odom', 'odom'),
            ('/scan', 'scan'),
            ('/stereo_camera/left/image_raw', 'stereo_camera/left/image_raw'),
            ('/stereo_camera/right/image_raw', 'stereo_camera/right/image_raw'),
            # ('/imu', 'imu'),
            ('/cmd_vel', 'cmd_vel'),
            # ('/tf', 'tf'),
            ('stereo_camera/left/camera_info', 'stereo_camera/left/camera_info'),
            ('stereo_camera/right/camera_info', 'stereo_camera/right/camera_info'),
        ]
    )
    

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(share_dir,
            'meshes'))

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(models_resource)
    ld.add_action(worlds_resource)

    ld.add_action(set_env_vars_resources)
    ld.add_action(rsp)
    ld.add_action(gz_sim)
    ld.add_action(iris_spawner)
    ld.add_action(ros_gz_bridge)

    return ld