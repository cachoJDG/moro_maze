import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    moro_maze_dir = get_package_share_directory('moro_maze')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    tb_nav2_bringup_dir = get_package_share_directory('turtlebot3_navigation2')
    
    # Configuration files.
    world_path = os.path.join(moro_maze_dir, 'worlds', 'default_gzsim.world')
    map_yaml_path = os.path.join(moro_maze_dir, 'maps', 'map.yaml')
    rviz_config_file = os.path.join(moro_maze_dir, 'rviz', 'config.rviz')

    params_file_path = os.path.join(tb_nav2_bringup_dir, 'param', 'burger.yaml')
    # params_file_path = os.path.join(moro_maze_dir, 'params', 'nav2_params.yaml')
    # The Humble parameter file causes a TF error on Jazzy, so the TurtleBot3
    # navigation parameters are used instead.
    
    # Gazebo simulation.
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world_path], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models'))

    # Robot spawn and state publisher.
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='2.0')
    y_pose = LaunchConfiguration('y_pose', default='1.0')

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Nav2 map server, AMCL, and RViz.
    nav2_bringup_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(nav2_launch_dir, 'bringup_launch.py')),
    launch_arguments={'namespace': "",
                      'use_namespace': "False",
                      'slam': "False",
                      'map': map_yaml_path,
                      'use_sim_time': use_sim_time,
                      'params_file': params_file_path,
                      'autostart': "True",
                      'use_composition': "True",
                      'use_respawn': "False"}.items())

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'rviz_launch.py')),
        launch_arguments={'namespace': "",
                        'use_namespace': "False",
                        'use_sim_time': use_sim_time,
                        'rviz_config': rviz_config_file}.items())

    localisation_node_cmd = Node(
        package='moro_maze',
        executable='localisation_node',
        name='localisation_node',
        prefix='python3',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    global_planner_node_cmd = Node(
        package='moro_maze',
        executable='global_planner_node',
        name='global_planner_node',
        prefix='python3',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # MORO local planner: forward-simulation controller that drives the robot
    # along the global path by publishing directly to /cmd_vel (replaces the
    # Nav2 navigate_to_pose-based local_controller_node, which is kept in the
    # package for reference / fallback).
    local_planner_node_cmd = Node(
        package='moro_maze',
        executable='local_planner_node',
        name='local_planner_node',
        prefix='python3',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        gzserver_cmd,  # Enable gzclient_cmd here for a graphical Gazebo client.
        spawn_turtlebot_cmd, robot_state_publisher_cmd, set_env_vars_resources,
        nav2_bringup_cmd, rviz_cmd,
        localisation_node_cmd, global_planner_node_cmd, local_planner_node_cmd,
    ])
