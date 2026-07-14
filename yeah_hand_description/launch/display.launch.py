import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('yeah_hand_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'yeah_hand.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'yeah_hand.rviz')

    # Process the Xacro file to URDF string
    # (This automatically resolves $(find ...) tags inside the files)
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_config.toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_xml}]
        ),
#        Node(
#            package='joint_state_publisher_gui',
#            executable='joint_state_publisher_gui',
#            name='joint_state_publisher_gui',
#            output='screen'
#        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
