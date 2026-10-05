from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
  return LaunchDescription([
      DeclareLaunchArgument(
          'linear_button',
          default_value='0',
          description='Zero-based controller button that triggers linear motion',
      ),
      Node(
          package='my_pi_nodes',
          executable='serial_bridge',
          name='serial_bridge',
          output='screen',
      ),
      Node(
          package='my_pi_nodes',
          executable='controller',
          name='controller',
          output='screen',
      ),
      Node(
          package='my_pi_nodes',
          executable='MotorCmd_Test',
          name='MotorCmd_Test',
          output='screen',
          parameters=[{
              'linear_button': LaunchConfiguration('linear_button'),
          }],
      ),
  ])
