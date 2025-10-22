from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Test parameters
    test_string_param_arg = DeclareLaunchArgument(
        'test_string_param',
        default_value='Hello Redundancy',
        description='Test string parameter'
    )
    
    test_int_param_arg = DeclareLaunchArgument(
        'test_int_param',
        default_value='42',
        description='Test integer parameter'
    )
    
    test_double_param_arg = DeclareLaunchArgument(
        'test_double_param',
        default_value='3.14159',
        description='Test double parameter'
    )
    
    test_bool_param_arg = DeclareLaunchArgument(
        'test_bool_param',
        default_value='true',
        description='Test boolean parameter'
    )
    
    # Test namespace
    test_namespace_arg = DeclareLaunchArgument(
        'test_namespace',
        default_value='/test_ns',
        description='Test namespace'
    )
    
    # Test use_sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    return LaunchDescription([
        test_string_param_arg,
        test_int_param_arg,
        test_double_param_arg,
        test_bool_param_arg,
        test_namespace_arg,
        use_sim_time_arg,
        
        Node(
            package='redundancy_test',
            executable='redundancy_publisher',
            name='redundancy_publisher',
            output='screen',
            redundancy=True,
            namespace=LaunchConfiguration('test_namespace'),
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'test_string_param': LaunchConfiguration('test_string_param')},
                {'test_int_param': LaunchConfiguration('test_int_param')},
                {'test_double_param': LaunchConfiguration('test_double_param')},
                {'test_bool_param': LaunchConfiguration('test_bool_param')},
                {'node_specific_param': 'publisher_specific_value'},
            ],
            remappings=[
                ('redundancy_topic', 'remapped_topic'),
            ]
        ),
        
        Node(
            package='redundancy_test',
            executable='redundancy_subscriber',
            name='redundancy_subscriber',
            output='screen',
            redundancy=True,
            namespace=LaunchConfiguration('test_namespace'),
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'test_string_param': LaunchConfiguration('test_string_param')},
                {'test_int_param': LaunchConfiguration('test_int_param')},
                {'test_double_param': LaunchConfiguration('test_double_param')},
                {'test_bool_param': LaunchConfiguration('test_bool_param')},
                {'node_specific_param': 'subscriber_specific_value'},
            ],
            remappings=[
                ('redundancy_topic', 'remapped_topic'),
            ]
        ),
    ])