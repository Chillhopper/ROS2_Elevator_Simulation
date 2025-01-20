import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path to the Xacro file
    xacro_file = os.path.join(
        get_package_share_directory('my_bot'),
        'description',
        'my_robot.urdf.xacro'
    )

    # Process the Xacro file to generate URDF
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # Path to the controller YAML file
    controller_config = os.path.join(
        get_package_share_directory('my_bot'),
        'config',
        'elevator_controllers.yaml'  # Ensure this matches your file name
    )

    # Robot State Publisher (publishes TF and robot_description)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # Joint State Publisher GUI (allows slider control in RViz)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # Launch RViz2 with the appropriate config file
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('my_bot'), 'rviz', 'my_bot_rviz_config.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Add Gazebo Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
    )

    # Add Gazebo Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_bot'
        ],
        output='screen'
    )

    # Spawn the controllers **AFTER Gazebo has loaded**
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    elevator_mover = Node(
    	package="my_bot",
    	executable="elevator_mover_node",
    	name="elevator_mover",
    	output="screen"
    	)

    return LaunchDescription([
        gazebo_launch,                  # Start Gazebo first
        rsp_node,                        # Publishes robot_description
        joint_state_publisher_node,      # Adds sliders for joint control
        rviz_node,                        # Launches RViz2
        spawn_entity,                     # Spawns the robot in Gazebo
        joint_state_broadcaster_spawner,  # Loads joint state broadcaster
        position_controller_spawner,      # Loads position controller
        elevator_mover
    ])

