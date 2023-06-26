import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution

#define various functions that are usefull for loading different file types
def get_path(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    return os.path.join(package_path, file_path)

def load_file(package_name, file_path):
    absolute_file_path = get_path(package_name, file_path)
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

    controllers_file = get_path("ur5_scan_path_service", "config/ur_controllers.yaml")
    print(controllers_file)
    
    ur_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ur_bringup'), 'launch'),
         '/ur_control.launch.py']),
        launch_arguments={'ur_type': TextSubstitution(text=str('ur5')), 'robot_ip': TextSubstitution(text=str('192.168.1.131')),
        'use_fake_hardware': TextSubstitution(text=str('false')), 'launch_dashboard_client': TextSubstitution(text=str('true')),'launch_rviz': TextSubstitution(text=str('false')),
        'runtime_config_package': TextSubstitution(text='ur5_scan_path_service'),'controllers_file': TextSubstitution(text='ur5_controllers.yaml')}.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ur_bringup'), 'launch'),
         '/ur_moveit.launch.py']),
        launch_arguments={'ur_type': TextSubstitution(text=str('ur5')), 'robot_ip': TextSubstitution(text=str('192.168.1.131')),
        'use_fake_hardware': TextSubstitution(text=str('false')), 'launch_rviz': TextSubstitution(text=str('true'))}.items(),
    )

    servo_yaml = load_yaml(
        "ur5_scan_path_service", "config/moveit_servo.yaml"
    )

    servo_params = {"moveit_servo": servo_yaml}

    scan_service = Node(
            package= 'ur5_scan_path_service',
            executable='ur5_scan_path_service',
            name='scan_service',
            output='screen',
            parameters = [robot_description, robot_description_semantic, kinematics_yaml, {'use_sim_time': True}]
        )

    moveit_servo = ComposableNodeContainer(
        name="moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[robot_description],
                extra_arguments=[{'use_intra_process_comms' : True}]
            ),
            ComposableNode(
                package= 'moveit_servo',
                plugin='moveit_servo::ServoServer',
                name='servo_node',
                parameters = [robot_description, robot_description_semantic, servo_params],
                extra_arguments=[{'use_intra_process_comms' : True}]
            ),
            ComposableNode(
                package= 'moveit_servo',
                plugin='moveit_servo::JoyToServoPub',
                name='joy_node',
                extra_arguments=[{'use_intra_process_comms' : True}]
            )
        ]

    )

    robot_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_group_position_controller", "-c", "/controller_manager","--stopped"],
    )
        
    return LaunchDescription([
        scan_service,
        ur_controller,
        moveit,
        moveit_servo,
        robot_position_controller_spawner
   ])