from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
       Node(
           package='camera_package',
           executable='image_processing_node',
           name='imageProcessing'
       ),
       Node(
           package='camera_package',
           executable='image_display_node',
           name='imageDisplay'
       ),
       Node(
           package='movement_controller',
           executable='controller_node',
           name='movementController'
       )
   ])