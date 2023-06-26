import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution

#define various functions that are usefull for loading different file types
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None
    
def generate_launch_description():
    #load the urdf file
    robot_description_config = load_file(
        "ur5_scan_path_service", "config/ur5.urdf"
    )

    robot_description = {"robot_description": robot_description_config}
    
    #load the srdf file
    robot_description_semantic_config = load_file(
         "ur5_scan_path_service", "config/ur5.srdf"
     )
    
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }
    
    #loads kinematics file. This file configures the settings for the inverse kinematics solver
    kinematics_yaml = load_yaml(
        "ur_moveit_config", "config/kinematics.yaml"
    )

    ur_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ur_bringup'), 'launch'),
         '/ur_control.launch.py']),
        launch_arguments={'ur_type': TextSubstitution(text=str('ur5')), 'robot_ip': TextSubstitution(text=str('192.168.1.131')),
        'use_fake_hardware': TextSubstitution(text=str('true')), 'launch_dashboard_client': TextSubstitution(text=str('true')),'launch_rviz': TextSubstitution(text=str('false'))}.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ur_bringup'), 'launch'),
         '/ur_moveit.launch.py']),
        launch_arguments={'ur_type': TextSubstitution(text=str('ur5')), 'robot_ip': TextSubstitution(text=str('192.168.1.131')),
        'use_fake_hardware': TextSubstitution(text=str('true')), 'launch_rviz': TextSubstitution(text=str('true'))}.items(),
    )
    return LaunchDescription([
        Node(
            package= 'ur5_scan_path_service',
            executable='ur5_scan_path_service',
            name='scan_service',
            output='screen',
            parameters = [robot_description, robot_description_semantic, kinematics_yaml],
            #prefix="gdbserver localhost:3000"
        ),
        ur_controller,
        moveit
   ])