from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter, SetRemap
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_stereo_image_proc = get_package_share_directory(
        'stereo_image_proc')

    # Paths
    stereo_image_proc_launch = PathJoinSubstitution(
        [pkg_stereo_image_proc, 'launch', 'stereo_image_proc.launch.py'])

    localization = LaunchConfiguration('localization')

    parameters={
          'frame_id':'camera_link',
          'subscribe_rgbd':False,
          'subscribe_stereo':True,
          'approx_sync':True, # odom is generated from images, so we can exactly sync all inputs
          'map_negative_poses_ignored':True,
          'subscribe_odom_info': True,
          'Stereo/MaxDisparity': '512',
          # RTAB-Map's internal parameters should be strings
          'OdomF2M/MaxSize': '1000',
          'GFTT/MinDistance': '5',
          'GFTT/QualityLevel': '0.01',
          'Kp/DetectorStrategy': '6', # Uncommment to match ros1 noetic results, but opencv should be built with xfeatures2d
          'Vis/FeatureType': '6'      # Uncommment to match ros1 noetic results, but opencv should be built with xfeatures2d
    }

    remappings=[
         ('odom_rgbd_image', '/stereo_camera/rgbd_image'),
         ('odom',       '/vo')]
    
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
    )

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument('rtabmap_viz',  default_value='true',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='true',   description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('localization', default_value='false',  description='Launch in localization mode.'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,  description='Configuration path of rviz2.'),

        SetParameter(name='use_sim_time', value=True),

        # Nodes to launch

        # Uncompress images for stereo_image_rect and remap to expected names from stereo_image_proc
        Node(
            package='image_transport', executable='republish', name='republish_left', output='screen',
            namespace='stereo_camera',
            arguments=['compressed', 'raw'],
            remappings=[('in/compressed', 'left/image_raw_throttle/compressed'),
                        ('out',           'left/image_raw')]),
        Node(
            package='image_transport', executable='republish', name='republish_right', output='screen',
            namespace='stereo_camera',
            arguments=['compressed', 'raw'],
            remappings=[('in/compressed', 'right/image_raw_throttle/compressed'),
                        ('out',           'right/image_raw')]),

        # Run the ROS package stereo_image_proc for image rectification   
        GroupAction(
            actions=[

                SetRemap(src='stereo_camera/left/camera_info',dst='stereo_camera/left/camera_info_throttle'),
                SetRemap(src='stereo_camera/right/camera_info',dst='stereo_camera/right/camera_info_throttle'),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([stereo_image_proc_launch]),
                    launch_arguments=[
                        ('left_namespace', 'stereo_camera/left'),
                        ('right_namespace', 'stereo_camera/right'),
                        # ('disparity_range', '192'),
                    ]
                ),
            ]
        ),
        
        # Synchronize stereo data together in a single topic
        # Issue: stereo_img_proc doesn't produce color and 
        #        grayscale images exactly the same (there is a small 
        #        vertical shift with color), we should use grayscale for 
        #        left and right images to get similar results than on ros1 noetic.
        Node(
            package='rtabmap_sync', executable='stereo_sync', output='screen',
            namespace='stereo_camera',
            remappings=[
                ('stereo_camera/left/image_rect',   'stereo_camera/left/image_rect'),
                ('stereo_camera/right/image_rect',  'stereo_camera/right/image_rect'),
                ('stereo_camera/left/camera_info',  'stereo_camera/left/camera_info_throttle'),
                ('stereo_camera/right/camera_info', 'stereo_camera/right/camera_info_throttle')]),

        # Visual odometry
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            namespace='stereo_camera',
            parameters=[parameters],
            remappings=remappings),
        
        # SLAM mode:
        # Node(
        #     condition=UnlessCondition(localization),
        #     package='rtabmap_slam', executable='rtabmap', output='screen',
        #     parameters=[parameters],
        #     remappings=remappings,
        #     arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # # Localization mode:
        # Node(
        #     condition=IfCondition(localization),
        #     package='rtabmap_slam', executable='rtabmap', output='screen',
        #     parameters=[parameters,
        #       {'Mem/IncrementalMemory':'False',
        #        'Mem/InitWMWithAllNodes':'True'}],
        #     remappings=remappings),

        # Visualization:
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            namespace='stereo_camera',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[parameters,
                        {"odometry_node_name": 'stereo_odometry'}],
            remappings=remappings),
        Node(
            package='rviz2', executable='rviz2', name="rviz2", output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]]),
    ])


   
   

