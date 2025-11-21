from dataclasses import dataclass


@dataclass
class HandGuidanceCustomConfig:
    """Configuration for hand guidance teleoperator."""
    teleop_type: str = "hand_guidance_custom"
    
    # Hand guidance parameters
    force_sensitivity: float = 0.01
    torque_sensitivity: float = 0.005
    gain_tr: float = 10.0
    gain_rot: float = 50.0
    alpha: float = 1.0
    f_threshold_high: float = 0.2
    f_threshold_low: float = 0.05
    m_threshold_high: float = 0.25
    m_threshold_low: float = 0.05