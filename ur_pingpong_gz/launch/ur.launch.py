import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ur_pingpong_share = get_package_share_directory('ur_pingpong_gz')
    ur_sim_share = get_package_share_directory('ur_simulation_gz')
    
    world_file = os.path.join(
        ur_pingpong_share,
        'worlds',
        'world.sdf'
    )

    ur_sim_gz = IncludeLaunchDescription(
        os.path.join(
            ur_sim_share,
            'launch',
            'ur_sim_control.launch.py'
        ),
        launch_arguments={
            'world_file': world_file,
            'launch_rviz': 'false',
            'initial_joint_controller': 'forward_position_controller',
            'description_package': 'ur_pingpong_gz',
            'description_file': 'ur_with_paddle.xacro.urdf'
        }.items()
    )

    camera_urdf = os.path.join(
        ur_pingpong_share,
        'urdf',
        'camera.urdf'
    )
    gz_spawn_camera = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file",
            camera_urdf,
            "-name",
            "camera",
            "-allow_renaming",
            "true"
        ],
    )

    camera_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="camera_state_publsiher",
        namespace="camera",
        output="both",
        parameters=[{"use_sim_time": True},
                    {'robot_description': open(camera_urdf).read()}
        ],
    )

    gz_sim_bridge = Node(
        name='camera_gz_bridge',
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output="screen",
    )

    rviz_config_file = os.path.join(
        ur_pingpong_share,
        'rviz',
        'rviz_config.rviz'
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        ur_sim_gz,
        gz_spawn_camera,
        gz_sim_bridge,
        camera_state_publisher,
        rviz_node
    ])
