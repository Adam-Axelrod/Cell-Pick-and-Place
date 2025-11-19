from dataclasses import dataclass
from lerobot.teleoperators.config import TeleoperatorConfig
import numpy as np

# Register this teleop as "hand_guidance_custom"
@TeleoperatorConfig.register_subclass("hand_guidance_custom")
@dataclass
class HandGuidanceCustomConfig(TeleoperatorConfig):
    # Add settings like sensitivity
    force_sensitivity: float = 0.01
    torque_sensitivity: float = 0.005

    # Hand guidance params
    gain_tr = 10
    gain_rot = 50
    alpha = 1
    f_threshold_high = 0.2
    f_threshold_low = 0.05
    m_threshold_high = 0.25
    m_threshold_low = 0.05
