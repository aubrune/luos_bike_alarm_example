import launch
import launch.actions
import launch_ros.actions
from os.path import join
from ament_index_python.packages import get_package_prefix
from ament_index_python.resources import RESOURCE_INDEX_SUBFOLDER

rviz_file = join(get_package_prefix('luos_bike_alarm_example'), RESOURCE_INDEX_SUBFOLDER, "packages", 'bike.rviz')

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='luos_interface', executable='broker', output='screen'),
        launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', rviz_file]),
        launch_ros.actions.Node(
            package='luos_bike_alarm_example', executable='bike_alarm', output='screen')
    ])
