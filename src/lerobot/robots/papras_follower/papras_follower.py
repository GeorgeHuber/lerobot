#!/usr/bin/env python

# Copyright 2025 George Huber All rights reserved.
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

from lerobot.cameras import make_cameras_from_configs
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.dynamixel import DynamixelMotorBus
from lerobot.robots import Robot

class PaprasFollower(Robot):
    config_class = PaprasFollowerConfig
    name = "papras_follower"

    def __init__(self, config: PaprasFollowerConfig):
        super().__init__(config)
        self.bus = DynamixelMotorsBus(
            port=self.config.port,
            motors={
                # @TODO: may need to expand motor range
                "joint1": Motor(1, "x_series", MotorNormMode.RANGE_M100_100, baudrate=460_800),
                "joint2": Motor(2, "x_series", MotorNormMode.RANGE_M100_100),
                "joint3": Motor(3, "x_series", MotorNormMode.RANGE_M100_100),
                "joint4": Motor(4, "x_series", MotorNormMode.RANGE_M100_100),
                "joint5": Motor(5, "x_series", MotorNormMode.RANGE_M100_100),
                "joint6": Motor(5, "x_series", MotorNormMode.RANGE_M100_100),
                "joint7": Motor(5, "x_series", MotorNormMode.RANGE_M100_100),
            },
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)
    
    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            "joint1.pos": float,
            "joint2.pos": float,
            "joint3.pos": float,
            "joint4.pos": float,
            "joint5.pos": float,
            "joint6.pos": float,
            "joint7.pos": float,
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras
        }

    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}
    
    def action_features(self) -> dict:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())
    
    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        if not self.is_calibrated and calibrate:
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        
    def disconnect(self) -> None:
        self.bus.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()
            
    def configure(self) -> None:
        with self.bus.torque_disabled():
            self.bus.configure_motors()
            for motor in self.bus.motors:
                self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                self.bus.write("P_Coefficient", motor, 16)
                self.bus.write("I_Coefficient", motor, 0)
                self.bus.write("D_Coefficient", motor, 32)
    
    def calibrate(self) -> None:
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move {self} to the middle of its range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings()

        print(
            "Move all joints sequentially through their entire ranges "
            "of motion.\nRecording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion()

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)
    
    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")

        # Read arm position
        obs_dict = self.bus.sync_read("Present_Position")
        obs_dict = {f"{motor}.pos": val for motor, val in obs_dict.items()}

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        return obs_dict
    
    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items()}

        # Send goal position to the arm
        self.bus.sync_write("Goal_Position", goal_pos)

        return action