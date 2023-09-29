import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController




def generate_launch_description():

    ld = LaunchDescription()

    # webots world
    package_dir = get_package_share_directory('hbird_webots')
    world = LaunchConfiguration('world')
    world_launch_arg = DeclareLaunchArgument(
            'world',
            default_value='LPB.wbt',
            description='Choose one of the world files from `/hbird_webots/worlds` directory'
        )
    
    ld.add_action(world_launch_arg)
    
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    ld.add_action(webots)
    ld.add_action(webots._supervisor)

    # webots description
    robot_description_path = os.path.join(package_dir, 'resource', 'hbird_drone.urdf')

    
    # agent and hbird_sim nodes
    agent_control_node = Node(
            package="hbird_navigation",
            executable="agent_control_node",
            output='screen',
            name='agent_control_node'
        )

    hbird_sim_node = WebotsController(
                robot_name='HB1',
                parameters=[
                    {'robot_description': robot_description_path,
                        'agent_id': 'HB1'},
                ],
                respawn=True
        )
    
        
    ld.add_action(agent_control_node)
    ld.add_action(hbird_sim_node)

    # event handler
    exit_event_handler = launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ld.add_action(exit_event_handler)
    


    return ld
