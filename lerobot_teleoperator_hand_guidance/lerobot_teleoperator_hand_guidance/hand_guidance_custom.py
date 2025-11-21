import numpy as np
from numpy import linalg as LA

from .config_hand_guidance_custom import HandGuidanceCustomConfig


class HandGuidanceCustom:
    """Hand guidance teleoperator using force sensor."""
    
    def __init__(self, config: HandGuidanceCustomConfig | None = None, **kwargs):
        if config is None:
            config = HandGuidanceCustomConfig()
        
        # Allow kwargs to override config
        for key, value in kwargs.items():
            if hasattr(config, key):
                setattr(config, key, value)
        
        self.config = config
        self.wrench = np.zeros(6)
        self.wrench_filtered = np.zeros(6)
        self.motion_group = "NONE"

    def get_action(self, robot_state):
        """
        Compute action from robot state (which includes force data).
        
        Args:
            robot_state: dict with 'force' key containing [fx, fy, fz, tx, ty, tz]
        
        Returns:
            action: 6-element array [dx, dy, dz, drx, dry, drz]
        """
        # Extract force data from robot state
        self.wrench = np.array(robot_state.get("force", [0.0] * 6))
        
        # Filter wrench
        self.wrench_filtered = (self.wrench_filtered + 
                                self.config.alpha * (self.wrench - self.wrench_filtered))
        
        # Compute action
        delta_pose_action = np.zeros(6)
        
        normF = LA.norm(self.wrench_filtered[0:3])
        normM = LA.norm(self.wrench_filtered[3:6])
        
        # Determine motion group
        if normF <= self.config.f_threshold_low and normM < self.config.m_threshold_low:
            self.motion_group = "NONE"
        elif normM > self.config.m_threshold_high:
            self.motion_group = "ROTATION"
        elif self.motion_group == "ROTATION" and normM > self.config.m_threshold_low:
            self.motion_group = "ROTATION"
        elif normF > self.config.f_threshold_high:
            self.motion_group = "TRANSLATION"
        elif self.motion_group == "TRANSLATION" and normF < self.config.f_threshold_low:
            self.motion_group = "NONE"
        elif self.motion_group == "ROTATION" and normM < self.config.m_threshold_low:
            self.motion_group = "NONE"
        
        # Compute delta based on motion group
        if self.motion_group == "TRANSLATION" and normF > 0:
            temp_deadzone = (self.wrench_filtered[0:3] / normF * self.config.f_threshold_low)
            delta_translation = self.config.gain_tr * (self.wrench_filtered[0:3] - temp_deadzone)
            delta_pose_action[0:3] = delta_translation
        elif self.motion_group == "ROTATION" and normM > 0:
            temp_deadzone = (self.wrench_filtered[3:6] / normM * self.config.m_threshold_low)
            delta_rotation = self.config.gain_rot * (self.wrench_filtered[3:6] - temp_deadzone)
            delta_pose_action[3:6] = delta_rotation
        
        # Clip values
        delta_pose_action[0:3] = np.clip(delta_pose_action[0:3], -60, 60)
        delta_pose_action[3:6] = np.clip(delta_pose_action[3:6], -45, 45)
        
        return delta_pose_action

    def disconnect(self):
        """Cleanup - nothing to do for this teleoperator."""
        pass