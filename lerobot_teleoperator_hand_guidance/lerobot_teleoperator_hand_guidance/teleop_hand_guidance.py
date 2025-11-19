import numpy as np
from lerobot.teleoperators.teleoperator import Teleoperator
from .config_hand_guidance import HandGuidanceCustomConfig

import numpy as np
from numpy import linalg as LA

@Teleoperator.register_subclass("hand_guidance_custom")
class HandGuidanceCustom(Teleoperator):
    config_class = HandGuidanceCustomConfig

    def __init__(self, config: HandGuidanceCustomConfig):
        super().__init__(config)
        self.config = config
        self.wrench = np.zeros(6)
        self.wrench_filtered = np.zeros(6)


    def get_action(self, observation):
        delta_pose_action = np.zeros(6)
        motiongroup = "NONE"
        # Get the force reading FROM THE ROBOT'S OBSERVATION
        self.wrench = observation["force"]  # Assuming observation contains 'force' key

        wrench_filtered = np.zeros(6)
        wrench_filtered = wrench_filtered + self.config.alpha * (self.wrench - wrench_filtered)

        normF = LA.norm(self.wrench[0:3])
        normM = LA.norm(self.wrench[3:])

        if normF <= self.config.f_threshold_low and normM < self.config.m_threshold_low:
            motiongroup = "NONE"

        if normM > self.config.m_threshold_high:
            motiongroup = "ROTATION"
        else:
            if motiongroup == "ROTATION" and normM > self.config.m_threshold_low:
                motiongroup = "ROTATION"
            elif normF > self.config.f_threshold_high:
                motiongroup = "TRANSLATION"
            elif motiongroup == "ROTATION" and normM < self.config.m_threshold_low:
                motiongroup = "NONE"
            elif motiongroup == "TRANSLATION" and normF < self.config.f_threshold_low:
                motiongroup = "NONE"
        
        match motiongroup:
            case ("NONE"):
                delta_pose_action = np.zeros(6)
            case ("TRANSLATION"):
                tempDeadzone = (wrench_filtered[0:3] / normF * self.config.f_threshold_low)
                delta_translation = self.config.gain_tr * (wrench_filtered[0:3] - tempDeadzone)
                delta_pose_action = [delta_translation[0], delta_translation[1], delta_translation[2], 0.0, 0.0, 0.0]
            case ("ROTATION"):
                tempDeadzone = (wrench_filtered[3:] / normM * self.config.m_threshold_low)
                delta_rotation = self.config.gain_rot * (wrench_filtered[3:] - tempDeadzone)
                delta_pose_action = [0.0, 0.0, 0.0, delta_rotation[0], delta_rotation[1], delta_rotation[2]]
            
        for i in range(3):
            delta_pose_action[i] = np.clip(delta_pose_action[i], -60, 60)
            delta_pose_action[i+3] = np.clip(delta_pose_action[i+3], -45, 45)

        return np.array(delta_pose_action)

    def close(self):
        # Nothing to clean up for this device
        pass