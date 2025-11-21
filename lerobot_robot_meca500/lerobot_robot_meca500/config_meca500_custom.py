from dataclasses import dataclass
from lerobot.robots.config import RobotConfig


@dataclass
class Meca500CustomConfig(RobotConfig):
    # Add any config you need, like the IP address

    robot_type: str = "meca500_custom"

    ip_address: str = "192.168.0.100"
    # We can also define the camera names lerobot should expect
    camera_names: list[str] = ("overhead_camera", "wrist_camera", "microscope_feed")
    
    sensor_config_path: str = "C:/Users/aa24/PhD/Cell-Pick-and-Place/lerobot_robot_meca500/lerobot_robot_meca500/sensor_config.json"
