from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  return LaunchDescription([
      Node(
          package='my_pi_nodes',
          executable='serial_bridge',
          name='serial_bridge',
          output='screen',
      ),
      Node(
          package='my_pi_nodes',
          executable='controller',
          name='Controller',
          output='screen',
      ),
      Node(
          package='my_pi_nodes',
          executable='Motor_Cmd',
          name='DeltaControl',
          output='screen',
      ),
  ])