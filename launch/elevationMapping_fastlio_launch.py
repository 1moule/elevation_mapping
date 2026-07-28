import os

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory("elevation_mapping")
    config_dir = os.path.join(share_dir, "config")
    rviz_config = os.path.join(share_dir, "rviz2", "fastlio_elevation.rviz")
    use_sim_time = launch.substitutions.LaunchConfiguration("use_sim_time")

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the clock published by rosbag playback.",
            ),
            launch_ros.actions.Node(
                package="elevation_mapping",
                executable="elevation_mapping",
                name="elevation_mapping",
                output="screen",
                parameters=[
                    os.path.join(config_dir, "robots", "fastlio_mid360.yaml"),
                    os.path.join(config_dir, "postprocessing", "postprocessor_pipeline.yaml"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="elevation_mapping_rviz",
                output="screen",
                arguments=["--display-config", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
