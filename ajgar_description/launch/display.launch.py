from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    arm_share_dir = get_package_share_directory('ajgar_description')
    table_share_dir = get_package_share_directory('fixture_table_description')

    arm_xacro_file = os.path.join(arm_share_dir, 'urdf', 'ajgar.xacro')
    robot_description_config = xacro.process_file(arm_xacro_file)
    robot_urdf = robot_description_config.toxml()

    table_xacro_file = os.path.join(table_share_dir, 'urdf', 'fixture_table.xacro')
    robot_description_config = xacro.process_file(table_xacro_file)
    table_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(arm_share_dir, 'config', 'display.rviz')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    table_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='table_state_publisher',
        parameters=[
            {'robot_description': table_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        # table_state_publisher_node
    ])
