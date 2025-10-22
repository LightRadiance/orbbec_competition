from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameters_type import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 使用仿真时间
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # world -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'odom']
        ),

        # base_link -> camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_camera_broadcaster',
            arguments=['0.0608', '0', '0.1213', '0', '0', '0', '1', 'base_link', 'camera']
        ),

        # camera -> camera_rgb
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_rgb_broadcaster',
            arguments=['0.0076121', '0.010989', '-0.0000076', '0', '0', '0', '1', 'camera', 'camera_rgb']
        ),

        # camera -> camera_left
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_left_broadcaster',
            arguments=['0.0061398', '-0.025', '0', '0', '0', '0', '1', 'camera', 'camera_left']
        ),

        # camera -> camera_right
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_right_broadcaster',
            arguments=['0.0061398', '0.025', '-0.0000001', '0', '0', '0', '1', 'camera', 'camera_right']
        ),

        # take_picture 节点
        Node(
            package='take_picture',
            executable='take_picture',
            name='take_picture',
            output='screen',
            parameters=[{
                'image_save_path': '/home/labor/Documents/research/orbbec_competition/image/',
                'video_save_path': '/home/labor/Documents/research/orbbec_competition/video/',
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # drone_drive 节点
        Node(
            package='drone_drive',
            executable='drone_drive',
            name='drone_drive',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])
