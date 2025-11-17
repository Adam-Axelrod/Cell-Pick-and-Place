import numpy as np
from lerobot.teleoperators.teleoperator import Teleoperator
from .config_hand_guidance import HandGuidanceCustomConfig

class HandGuidanceCustom(Teleoperator):
    config_class = HandGuidanceCustomConfig

    def __init__(self, config: HandGuidanceCustomConfig):
        super().__init__(config)
        self.config = config

    def get_action(self, observation):
        # Get the force reading FROM THE ROBOT'S OBSERVATION
        force_reading = observation["force"]

        # TODO: Add your algorithm that turns `force_reading` 
        # into a 6D delta-pose

        # Example placeholder:
        # delta_x = force_reading[0] * self.config.force_sensitivity
        # ... etc ...
        # delta_pose_action = [delta_x, delta_y, delta_z, delta_rx, delta_ry, delta_rz]

        # REPLACE THIS with your real logic
        delta_pose_action = np.zeros(6) 

        return np.array(delta_pose_action)

    def close(self):
        # Nothing to clean up for this device
        pass