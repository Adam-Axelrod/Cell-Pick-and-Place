"""Custom recording script for Meca500 with hand guidance."""
import time
import numpy as np
from pathlib import Path

from lerobot_robot_meca500.lerobot_robot_meca500 import Meca500Custom, Meca500CustomConfig
from lerobot_teleoperator_hand_guidance.lerobot_teleoperator_hand_guidance import (
    HandGuidanceCustom, HandGuidanceCustomConfig
)

def main():
    print("Initializing Meca500 with hand guidance...")
    
    # Create configurations
    robot_config = Meca500CustomConfig(
        ip_address="192.168.0.100"
    )
    teleop_config = HandGuidanceCustomConfig()
    
    # Initialize robot and teleoperator
    robot = Meca500Custom(config=robot_config)
    teleop = HandGuidanceCustom(config=teleop_config)
    
    print("\nStarting hand guidance control loop...")
    print("Press Ctrl+C to stop.\n")
    
    try:
        control_freq = 10  # Hz
        dt = 1.0 / control_freq
        
        while True:
            start_time = time.time()
            
            # Get current state (includes force data)
            state = robot.get_state()
            
            # Get action from teleoperator
            action = teleop.get_action(state)
            
            # Send action to robot
            robot.send_action(action)
            
            # Maintain control frequency
            elapsed = time.time() - start_time
            if elapsed < dt:
                time.sleep(dt - elapsed)
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        robot.disconnect()
        teleop.disconnect()
        print("Disconnected successfully.")

if __name__ == "__main__":
    main()