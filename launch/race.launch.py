#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    package_name = 'rl_race'
    world_file_name = 'empty.sdf'
    urdf_file_name = 'F1race.urdf'
    
    package_dir = get_package_share_directory(package_name)
    world_file = LaunchConfiguration('world_file', default=join(package_dir, 'worlds', world_file_name))
    urdf_file = LaunchConfiguration('urdf_file', default=join(package_dir, 'urdf', urdf_file_name))

    position_x = LaunchConfiguration('position_x', default='0.0')
    position_y = LaunchConfiguration('position_y', default='0.0')
    position_z = LaunchConfiguration('position_z', default='3.0')
    orientation_yaw = LaunchConfiguration('orientation_yaw', default='0.0')

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', urdf_file])}]
    )


    # Joint state publisher node
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    # Include the Gazebo simulation launch file
    gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PythonExpression(["'", world_file, " -r'"])}.items()
    )

    # Spawn the robot in Ignition Gazebo
    gz_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-topic', '/robot_description',
                   '-name', 'my_robot',
                   '-allow_renaming', 'true',
                   '-z', position_z,
                   '-x', position_x,
                   '-y', position_y,
                   '-Y', orientation_yaw]
    )

    # Bridge between ROS 2 and Ignition Gazebo
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/world/empty_world/model/my_robot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'
        ],
        remappings=[
            ('/world/empty_world/model/my_robot/joint_state', '/joint_states')
        ]
    )

    # Static transform publisher (example)
    transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )

    # Map builder node
    map_builder = Node(
        package='rl_race',
        executable='map_builder',
        name='map_builder',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # TF broadcaster for odom to map
    odom_to_map_broadcaster = Node(
        package='rl_race',
        executable='odom_to_map_broadcaster.py',
        name='odom_to_map_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('world_file', default_value=world_file, description='Full path to the world file to load'),
        DeclareLaunchArgument('urdf_file', default_value=urdf_file, description='Full path to the URDF file to load'),
        DeclareLaunchArgument('position_x', default_value='0.0', description='Robot initial position x'),
        DeclareLaunchArgument('position_y', default_value='0.0', description='Robot initial position y'),
        DeclareLaunchArgument('position_z', default_value='0.5', description='Robot initial position z'),
        DeclareLaunchArgument('orientation_yaw', default_value='1.57', description='Robot initial orientation yaw'),

        gz_sim,
        robot_state_publisher,
        gz_spawn_entity,
        gz_ros2_bridge,
        transform_publisher,
        # joint_state_publisher,
        # map_builder,
        # odom_to_map_broadcaster
    ])
