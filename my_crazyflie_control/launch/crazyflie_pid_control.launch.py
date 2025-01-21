from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    
    sdf_path = os.path.join(get_package_share_path("my_crazyflie_control"), 'sdf', 'crazyflie_world.sdf')
    pkg_project_bringup = get_package_share_directory('my_crazyflie_control')
    pkg_project_gazebo = get_package_share_directory('my_robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_model_path = os.getenv('GZ_SIM_RESOURCE_PATH')

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(gz_model_path, 'crazyflie', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'crazyflie_world.sdf -r'
        ])}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_crazyflie_bridge.yaml'),
        }],

        output='screen'
    )

    control = Node(
        package='ros_gz_crazyflie_control',
        executable='control_pid',
        output='screen',
        parameters=[]
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        control
        
    ])