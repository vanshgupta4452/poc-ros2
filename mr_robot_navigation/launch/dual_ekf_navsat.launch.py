import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory
import launch.actions



def generate_launch_description():
    share_dir = get_package_share_directory('mr_robot_navigation')
    rl_params_file=os.path.join(share_dir,'config','params.yaml')

    navsat_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[rl_params_file, {"use_sim_time": True}],
        remappings=[
            ("imu/data", "imu/data"),
            ("gps/fix", "gps/fix"),
            ("gps/filtered", "gps/filtered"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/global"),
        ]
    )

    ekf_filter_node_odom = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[rl_params_file, {"use_sim_time": True}],
        remappings=[("odometry/filtered", "odometry/local")],
    )

    ekf_filter_node_map = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[rl_params_file, {"use_sim_time": True}],
        remappings=[("odometry/filtered", "odometry/global")],
    )


    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time', default_value='True',
            description='Flag to enable use_sim_time'
        ),
        # launch.actions.DeclareLaunchArgument(
        #     "output_final_position", default_value="false"
        # ),
        # launch.actions.DeclareLaunchArgument(
        #     "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
        # ),
        ekf_filter_node_odom,
        ekf_filter_node_map,
        navsat_node,
    ])
