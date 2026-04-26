import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# two separate containers to allow the client two input data using stdin, however this way memory isn't shared anymore

def generate_launch_description():
    user_interface_node = Node(
        package='assignment_1_rt2',
        executable='user_interface',
        name='user_interface',
        output='screen',
        parameters=[{'use_sim_time': True}],
        prefix='xterm -e'
    )
    server_container = ComposableNodeContainer(
        name='server_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='assignment_1_rt2',
                plugin='assignment_1_rt2::NavigationActionServer',
                name='service',
            ),
            ComposableNode(
                package='assignment_1_rt2',
                plugin='assignment_1_rt2::NavigationActionClient',
                name='client',
            ),
            ComposableNode(
                package='assignment_1_rt2',
                plugin='assignment_1_rt2::FramePublisher',
                name='goal_frame_publisher',
                parameters=[
                    {'topic_name': 'goal_frame'}
                ]
            ),
            ComposableNode(
                package='assignment_1_rt2',
                plugin='assignment_1_rt2::FramePublisher',
                name='robot_frame_publisher',
                parameters=[
                    {'topic_name': 'odom'}
                ]
            )
        ],
        output='screen',
        emulate_tty=True,
        prefix='xterm -e',
    )

    return launch.LaunchDescription([user_interface_node, server_container])