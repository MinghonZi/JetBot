"""
https://qiita.com/porizou1/items/70cd05be433ad02b7423
https://github.com/ros-planning/navigation2/tree/main/nav2_bringup
https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/launch/navigation_launch.py
https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/launch/bringup_launch.py
"""

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    nav2_params = 'params/nav2.yaml'

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    navi_nodes = GroupAction(
        actions = [
            # https://get-help.robotigniteacademy.com/t/what-is-the-component-container-isolated-used-for/17753
            Node(
                name = 'nav2_container',
                package = 'rclcpp_components',
                executable = 'component_container_isolated',
                parameters = [nav2_params],
                arguments = ['--ros-args', '--log-level', "info"],
                remappings = remappings,
                output = 'screen'),

            LoadComposableNodes(
                target_container = 'nav2_container',
                composable_node_descriptions = [
                    ComposableNode(
                        package = 'nav2_controller',
                        plugin = 'nav2_controller::ControllerServer',
                        name = 'controller_server',
                        parameters = [nav2_params],
                        remappings = remappings + [('cmd_vel', 'cmd_vel_nav')]),
                    ComposableNode(
                        package = 'nav2_smoother',
                        plugin = 'nav2_smoother::SmootherServer',
                        name = 'smoother_server',
                        parameters = [nav2_params],
                        remappings = remappings),
                    ComposableNode(
                        package = 'nav2_planner',
                        plugin = 'nav2_planner::PlannerServer',
                        name = 'planner_server',
                        parameters = [nav2_params],
                        remappings = remappings),
                    ComposableNode(
                        package = 'nav2_behaviors',
                        plugin = 'behavior_server::BehaviorServer',
                        name = 'behavior_server',
                        parameters = [nav2_params],
                        remappings = remappings),
                    ComposableNode(
                        package = 'nav2_bt_navigator',
                        plugin = 'nav2_bt_navigator::BtNavigator',
                        name = 'bt_navigator',
                        parameters = [nav2_params],
                        remappings = remappings),
                    ComposableNode(
                        package = 'nav2_waypoint_follower',
                        plugin = 'nav2_waypoint_follower::WaypointFollower',
                        name = 'waypoint_follower',
                        parameters = [nav2_params],
                        remappings = remappings),
                    ComposableNode(
                        package = 'nav2_velocity_smoother',
                        plugin = 'nav2_velocity_smoother::VelocitySmoother',
                        name = 'velocity_smoother',
                        parameters = [nav2_params],
                        remappings = remappings +
                                    [('cmd_vel', 'cmd_vel_nav'),
                                     ('cmd_vel_smoothed', 'cmd_vel')]),
                    ComposableNode(
                        package = 'nav2_lifecycle_manager',
                        plugin = 'nav2_lifecycle_manager::LifecycleManager',
                        name = 'lifecycle_manager_navigation',
                        parameters = [{'autostart': True,
                                       'node_names': [
                                            'bt_navigator',
                                            'controller_server',
                                            'planner_server',
                                            'smoother_server',
                                            'behavior_server',
                                            'waypoint_follower',
                                            'velocity_smoother']}]),
                ],
            )
        ]
    )

    return LaunchDescription([
        navi_nodes
    ])
