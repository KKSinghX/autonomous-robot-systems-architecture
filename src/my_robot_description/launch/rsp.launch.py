import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Define the package and file name
    pkg_name = 'my_robot_description'
    urdf_file = 'my_robot.urdf'

    # 2. Get the full path to the URDF file
    # Note: We look in 'install' because that's where ROS runs from
    urdf_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', urdf_file)

    # 3. Read the URDF file content
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 4. Create the Robot State Publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 5. Create the Joint State Publisher GUI node
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        # node_joint_state_publisher_gui
    ])