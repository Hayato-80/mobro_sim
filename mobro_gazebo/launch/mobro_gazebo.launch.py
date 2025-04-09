import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


def generate_launch_description():
    # File paths for URDF and RViz configuration
    description_pkg = os.path.join(get_package_share_directory('mobro_description'))
    gazebo_pkg = get_package_share_directory('mobro_gazebo')
    
    xacro_file = os.path.join(description_pkg,'urdf','robot.urdf.xacro')
    #robot_description_config = xacro.process_file(xacro_file)
    
    
    rviz_config_file = os.path.join(description_pkg, 'rviz', 'gazebo.rviz')
    gazebo_world_file = os.path.join(gazebo_pkg, 'worlds', 'test_home.world')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    #robot_description = Command(["xacro ",xacro_file])

    #robot_description_config = xacro.process_file(xacro_file)
    # robot_description_str = robot_description_config.toxml()
    # robot_description_param = ParameterValue(robot_description_str, value_type=str)
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': use_sim_time}

    gazebo_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]))
                    # launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()

    robot_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # name='spawn_entity',
        output='screen',
        # parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-entity', 'mobro', 
                   '-topic', '/robot_description'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Joint State Publisher (GUI) node
    # joint_state_publisher_gui_node = launch_ros.actions.Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=launch.conditions.IfCondition(LaunchConfiguration('use_gui'))
    # )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # RViz2 node with configuration file
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    joint_state_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broad'],
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diff_drive_controller'],
    )

    # Ensure joint_state_broad_spawner starts after gazebo_node
    joint_state_broad_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_spawn,
            on_exit=[joint_state_broad_spawner],
        )
    )

    # Ensure diff_drive_spawner starts after joint_state_broad_spawner
    diff_drive_spawner_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_spawn,
            on_exit=[diff_drive_spawner],
        )
    )

    # Return the launch description with all the nodes and the launch argument
    return LaunchDescription([
        # use_gui_arg,
        DeclareLaunchArgument(
          'world',
          default_value=[gazebo_world_file, ''],
          description='SDF world file'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        
        gazebo_node,
        robot_spawn,
        #joint_state_publisher,
        # joint_state_broad_spawner,
        joint_state_broad_after_gazebo,
        # diff_drive_spawner,
        diff_drive_spawner_after_joint_state,
        robot_state_publisher_node,
        rviz2_node,
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(slam_package_dir, 'launch',
        #                      'online_async_launch.py')),
        #     launch_arguments=[('slam_params_file', slam_params)])
    ])
