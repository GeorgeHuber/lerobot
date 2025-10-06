#!/usr/bin/env python

# Copyright 2025 George Huber. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import RobotConfig

from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
from lerobot.cameras.configs import ColorMode, Cv2Rotation


# # Instantiate and connect a `RealSenseCamera` with warm-up read (default).
# camera = RealSenseCamera(config)
# camera.connect()

# # Capture a color frame via `read()` and a depth map via `read_depth()`.
# try:
#     color_frame = camera.read()
#     depth_map = camera.read_depth()
#     print("Color frame shape:", color_frame.shape)
#     print("Depth map shape:", depth_map.shape)
# finally:
#     camera.disconnect()

@RobotConfig.register_subclass("papras_follower")
@dataclass
class PaprasFollowerConfig(RobotConfig):
    port: str
    # Per https://github.com/uiuckimlab/PAPRLE/blob/main/configs/follower/papras_7dof_2arm_table.yaml 
    # top:
    # serial_number: '233522070873'
    # rgb_resolution: [424,240]
    # depth_resolution: [480, 270]
    # get_aligned: false
    # depth_units: 0.000025
    # cameras: dict[str, CameraConfig] = field(
    # Maybe https://github.com/uiuckimlab/PAPRLE/blob/main/configs/follower/papras_teleop_bag.yaml 
        default_factory={
            
            "top": # Create a `RealSenseCameraConfig` specifying your camera’s serial number and enabling depth.
                RealSenseCameraConfig(
                    serial_number_or_name="2233522070873",
                    fps=15,
                    width=424,
                    height=240,
                    color_mode=ColorMode.RGB,
                    use_depth=True,
                    rotation=Cv2Rotation.NO_ROTATION
                )
        }
    )