import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

namespace_ = 'exomy'

def generate_launch_description():
    # Get the path to the exomy.yaml parameter file
    exomy_config = os.path.join(get_package_share_directory('exomy'),'exomy.yaml')
    print("exomy config: " + str(exomy_config))

    robot = Node(
        package='exomy',
        executable='robot_node',
        name='robot_node',
        namespace=namespace_,
        parameters=[exomy_config],
        output='screen'
    )
    joy = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=namespace_,
        output='screen'
    )
    joystick = Node(
        package='exomy',
        executable='joystick_parser_node',
        name='joystick_parser_node',
        namespace=namespace_,
        output='screen'
    )
    motors = Node(
        package='exomy',
        executable='motor_node',
        name='motor_node',
        namespace=namespace_,
        parameters=[exomy_config],
        output='screen'
    )

    return LaunchDescription([
        robot,
        joystick,
        joy,
        motors
    ])
