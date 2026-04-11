import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# two separate containers to allow the client two input data using stdin, however this way memory isn't shared anymore

def generate_launch_description():
    
    action_container = ComposableNodeContainer(
        name='action_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='assignment1_rt2',
                plugin='assignment1_rt2::NavigationActionServer',
                name='service',
            ),
            ComposableNode(
                package='assignment1_rt2',
                plugin='assignment1_rt2::NavigationActionClient',
                name='client',
            ),

        ],
        output='screen',
        emulate_tty=True,
        prefix='xterm -e',
    )
    
    user_interface_container = ComposableNodeContainer(
        name='user_interface_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='assignment1_rt2',
                plugin='assignment1_rt2::UserInterface',
                name='user_interface',
            ),
            ComposableNode(
                package='assignment1_rt2',
                plugin='assignment1_rt2::FramePublisher',
                name='goal_frame_publisher',
                parameters=[
                    {'topic_name': 'goal_frame'}
                ]
            ),
            ComposableNode(
                package='assignment1_rt2',
                plugin='assignment1_rt2::FramePublisher',
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

    return launch.LaunchDescription([action_container, user_interface_container])