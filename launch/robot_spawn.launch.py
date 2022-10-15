
import os, yaml, xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import Command, FindExecutable


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None



def generate_launch_description():
    # Import the model urdf (load from file, xacro ...)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("f1tenth_gym_ros"), "description", "sam_bot.xacro"]
            ),
        ]
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("f1tenth_gym_ros"), "description", "rviz.rviz"]
    )

    robot_description = {"robot_description": robot_description_content}
    

    # Robot state publisher
    params = {'use_sim_time': True}
    
 
    robot_state_publisher =  Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params,robot_description],
            arguments=[])

    # Ignition gazebo
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={'ign_args': '-r aapstrack.world'}.items(),
    )

    # RViz
    pkg_ros_ign_gazebo_demos = get_package_share_directory('ros_ign_gazebo_demos')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
    )

    # Spawn
    spawn = Node(package='ros_ign_gazebo', executable='create',
                 arguments=[
                    '-name', 'racecar',
                    '-x', '1.2',
                    '-z', '2.3',
                    '-Y', '3.4',
                    '-topic', '/robot_description'],
                 output='screen')

    ign_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
                #    '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                #    '/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                #    '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                   '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                   '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                #    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                #    '/world/empty/model/rrbot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
                #    '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'

                #    '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
                #    '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'
                   ],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",

    )

    ld = LaunchDescription()

    ld.add_action(robot_state_publisher)
    ld.add_action(rviz)
    ld.add_action(gazebo)
    ld.add_action(spawn)
    ld.add_action(joint_state_publisher)
    # ld.add_action(ign_bridge)

    return ld