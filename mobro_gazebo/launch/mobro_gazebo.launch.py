import os


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command
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
    base_pkg = get_package_share_directory('mobro_base')
    
    xacro_file = os.path.join(description_pkg,'urdf','robot.urdf.xacro')
    #robot_description_config = xacro.process_file(xacro_file)
    
    
    rviz_config_file = os.path.join(description_pkg, 'rviz', 'gazebo.rviz')
    gazebo_world_file = os.path.join(gazebo_pkg, 'worlds', 'test_home.world')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')

    # robot_description_config = xacro.process_file(xacro_file)
    # robot_description_str = robot_description_config.toxml()
    # robot_description_param = ParameterValue(robot_description_str, value_type=str)
    robot_description = Command(['xacro ',xacro_file, ' use_ros2_control:=', use_ros2_control])
    params = {'robot_description': robot_description, 'use_sim_time': use_sim_time}

    # doc = xacro.parse(open(xacro_file))
    # xacro.process_doc(doc)
    # params = {'robot_description': doc.toxml(), 'use_sim_time': use_sim_time}

    gazebo_parameters = os.path.join(gazebo_pkg, 'config', 'gazebo_parameters.yaml')

    gazebo_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={'args': '--ros-args --params-file ' + gazebo_parameters}.items()
    )

    robot_spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # name='spawn_entity',
        output='screen',
        arguments=['-entity', 'mobro', 
                   '-topic', '/robot_description'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )
    twist_mux_parameters = os.path.join(base_pkg ,'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_parameters, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_out','/diff_drive_controller/cmd_vel_unstamped')]
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
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2 control if true'
        ),
        
        gazebo_node,
        robot_spawn,
        twist_mux,
        #joint_state_publisher,
        # joint_state_broad_spawner,
        joint_state_broad_after_gazebo,
        # diff_drive_spawner,
        diff_drive_spawner_after_joint_state,
        robot_state_publisher_node,
        rviz2_node,
    ])
