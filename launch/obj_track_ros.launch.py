# SPDX-FileCopyrightText: 2025 Fabian Wieczorek, German Aerospace Center (DLR)
# SPDX-License-Identifier: MIT

import os
import launch
import launch_ros
from launch_param_builder import ParameterBuilder


def generate_launch_description():

    path = ParameterBuilder("obj_track_ros")._package_path

    node = launch_ros.actions.Node(
        package="obj_track_ros",
        executable="object_tracking_node",
        output="both",
        parameters=[
            {"camera_configs": [f"{path}/config/cameras.yaml"]},
            {"base_frame": "world"},
        ],
    )

    return launch.LaunchDescription([node])
