import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory("elevation_mapping")
    config_dir = os.path.join(share_dir, "config")
    rviz_config = os.path.join(share_dir, "rviz2", "fastlio_elevation.rviz")

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="elevation_mapping",
                executable="elevation_mapping",
                name="elevation_mapping",
                output="screen",
                parameters=[
                    os.path.join(config_dir, "robots", "fastlio_mid360.yaml"),
                    os.path.join(config_dir, "postprocessing", "postprocessor_pipeline.yaml"),
                ],
            ),
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="elevation_mapping_rviz",
                output="screen",
                arguments=["--display-config", rviz_config],
            ),
        ]
    )
