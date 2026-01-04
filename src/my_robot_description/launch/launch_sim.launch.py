import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_description'

    # 1. Include the Robot State Publisher (your previous launch file)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # # 2. Include the Gazebo launch file (provided by gazebo_ros)
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    # )

    # # 3. Run the spawner node
    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'my_cool_robot'],
    #                     output='screen')

    # ... inside generate_launch_description ...

    # 1. Ignition Gazebo Launch
    # 'empty.sdf' is the default world in Ignition
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(), # -r makes it start running immediately
    )

    # 2. Spawning the robot (Ignition uses 'create' instead of 'spawn_entity')
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_robot',
                                   '-allow_renaming', 'true'],
                        output='screen')

    # 3. THE BRIDGE (Critical!)
    # Ignition doesn't talk to ROS 2 topics automatically. We must map them.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/my_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/world/empty/model/my_robot/link/base_footprint/sensor/laser/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/world/empty/model/my_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/model/my_robot/odometry', '/odom'),
            ('/world/empty/model/my_robot/joint_state', '/joint_states'),
            ('/world/empty/model/my_robot/link/base_footprint/sensor/laser/scan', '/scan'),
        ],
        output='screen'
    )


    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        bridge,
    ])