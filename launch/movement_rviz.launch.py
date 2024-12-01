from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Controller node
        Node(
            package='movement_controller',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),
        # RViz node
        #Node(
            #package='rviz2',
            #executable='rviz2',
            #name='rviz2',
            #arguments=['-d', '/home/bojan/ros2_movement_ws_py/src/movement_controller/resource/rviz_config.rviz'],
            #output='screen'
        #),
        # Static transform publisher for base_link -> wheel_left
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.2', '-0.1', '0', '0', '0', 'base_link', 'wheel_left'],
            name='static_tf_base_to_wheel_left'
        ),
        # Static transform publisher for base_link -> wheel_right
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '-0.2', '-0.1', '0', '0', '0', 'base_link', 'wheel_right'],
            name='static_tf_base_to_wheel_right'
        )
    ])
