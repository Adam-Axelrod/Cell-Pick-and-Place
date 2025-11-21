# Correct import paths - note the double package name
from lerobot_robot_meca500.lerobot_robot_meca500 import Meca500Custom, Meca500CustomConfig
from lerobot_teleoperator_hand_guidance.lerobot_teleoperator_hand_guidance import HandGuidanceCustom, HandGuidanceCustomConfig

# Now run the lerobot-record command
import sys
from lerobot.scripts.lerobot_record import record

if __name__ == "__main__":
    sys.argv = [
        "record",
        "--robot.type", "meca500_custom",
        "--teleop.type", "hand_guidance_custom",
        "--dataset.repo_id", "AdamAxelrod/meca500-pipette-dataset",
        "--env.control_freq", "10",
        '--robot.cameras', '{"overhead_camera": {"type": "opencv", "index_or_path": 1, "width": 640, "height": 480}, "wrist_camera": {"type": "opencv", "index_or_path": 0, "width": 320, "height": 240}, "microscope_feed": {"type": "opencv", "index_or_path": 2, "width": 800, "height": 600}}'
    ]
    record()