from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Find package share directory
    pkg_kaliber_bt = get_package_share_directory('kaliber_behavior_tree')

    # Path to bt xml file
    # xml_file = os.path.join(pkg_q9_bt, 'bt_xml', 'q9_search.xml')

    # Locate the nav2_bringup package
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Full path to the target launch file
    tb3_sim_launch = os.path.join(nav2_bringup_dir, 'launch', 'tb3_simulation_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_sim_launch),
            launch_arguments={
                'headless': 'False',  # override headless argument
                'params_file': os.path.join(pkg_kaliber_bt, 'config', 'nav2_params.yaml')
            }.items()
        ),
        # Node(
        #     package='q9_bt',
        #     executable='dummy_server',
        #     name='dummy_server',
        #     output='screen',
        # ),
        # Node(
        #     package='q9_bt',
        #     executable='bt_runner_node',
        #     name='bt_runner_node',
        #     output='screen',
        #     parameters=[{
        #         'bt_xml_dir': xml_file
        #         }],
        # )
    ])
