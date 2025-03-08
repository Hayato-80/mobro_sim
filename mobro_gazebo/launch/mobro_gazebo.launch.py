import os
import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # File paths for URDF and RViz configuration
    # urdf_file = os.path.join(get_package_share_directory('mobro_description'), 'urdf', 'mobro.urdf')
    urdf_file = os.path.join(get_package_share_directory('mobro_description'), 'urdf', 'mobro_v2.urdf')
    rviz_config_file = os.path.join(get_package_share_directory('mobro_description'), 'rviz', 'gazebo.rviz')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_my_gazebo = get_package_share_directory('mobro_gazebo')
    gazebo_world_file = os.path.join(pkg_my_gazebo, 'worlds', 'empty.world')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Declare the launch argument for GUI visibility
    #use_gui_arg = DeclareLaunchArgument('use_gui', default_value='True', description='Use joint state publisher GUI')

    # gazebo_node = launch_ros.actions.Node(
    #     package='gazebo_ros',
    #     executable='gazebo',
    #     name='gazebo',
    #     arguments=['-s', 'libgazebo_ros_factory.so', gazebo_world_file]
    # )
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    robot_spawn = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # name='spawn_entity',
        output='screen',
        arguments=['-entity', 'mobro', '-topic', '/robot_description'],
    )

    # Joint State Publisher (GUI) node
    # joint_state_publisher_gui_node = launch_ros.actions.Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=launch.conditions.IfCondition(LaunchConfiguration('use_gui'))
    # )

    joint_state_publisher = launch_ros.actions.Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    parameters=[{'use_sim_time': use_sim_time}])

    # Robot State Publisher node
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    # RViz2 node with configuration file
    rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Return the launch description with all the nodes and the launch argument
    return LaunchDescription([
        # use_gui_arg,
        DeclareLaunchArgument(
          'world',
          default_value=[gazebo_world_file, ''],
          description='SDF world file'),
        gazebo_node,
        robot_spawn,
        joint_state_publisher,
        #joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz2_node,
    ])
