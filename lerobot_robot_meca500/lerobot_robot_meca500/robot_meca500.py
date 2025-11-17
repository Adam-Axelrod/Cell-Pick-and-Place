import mecademicpy.robot as robot
# TODO: Import your force sensor library

from lerobot.robots.robot import Robot
from .config_meca500 import Meca500CustomConfig

class Meca500Custom(Robot):
    # Tell lerobot which config class to use
    config_class = Meca500CustomConfig

    def __init__(self, config: Meca500CustomConfig):
        super().__init__(config)
        self.config = config

        print(f"Connecting to Meca500 at {self.config.ip_address}...")

        # --- 1. CONNECT TO HARDWARE ---
        # We instantiate the robot object from the imported library
        self.robot = robot.Robot()
        
        try:
            # Use connection, activation, and homing logic from adams_functions.py
            self.robot.Connect(address=self.config.ip_address, enable_monitor_mode=True)
            self.robot.ActivateRobot()
            self.robot.Home()
            self.robot.WaitHomed()
            
            # Set a default joint velocity (can be changed later)
            self.robot.SetJointVel(5) 
            
            # Apply the calibration settings from adams_functions.py
            print("Calibrating robot reference frames...")
            self.robot.SetTrf(193.448,-110.25,40.076,0,0,-30)
            self.robot.SetWrf(263.999238, 44.275, 158.61405, 0, 0, 0)
            print("Calibration complete.")

        except robot.MecademicException as e:
            print(f"An error occurred during robot initialization: {e}")
            raise

        # TODO: Add your 6-DOF force sensor connection code here
        self.force_sensor = None # Placeholder
        # Example:
        # try:
        #     self.force_sensor = my_force_sensor_lib.Sensor(port="/dev/ttyUSB0")
        # except Exception as e:
        #     print(f"Failed to connect to force sensor: {e}")
        #     self.force_sensor = None

        print("Hardware connected successfully.")

    def get_observation(self):
        # Get joint_angles from self.robot.GetJoints()
        # GetJoints() returns a tuple of 6 joint angles
        joint_angles = self.robot.GetJoints()

        # TODO: Get force_data from self.force_sensor.read()
        force_data = [0.0] * 6 # Placeholder: [fx, fy, fz, tx, ty, tz]

        if self.force_sensor:
            try:
                force_data = self.force_sensor.read()
            except Exception as e:
                print(f"Warning: Could not read from force sensor: {e}")
                force_data = [0.0] * 6 # Return zeros on error

        obs = {
            "state": joint_angles, 
            "force": force_data,    # We send force data to the teleop plugin
        }
        return obs

    def set_action(self, action):
        # Send a *relative* pose command
        # We assume 'action' is a 6-element list/array: [dx, dy, dz, d_alpha, d_beta, d_gamma]
        # We use MoveLinRelTrf to move relative to the current tool reference frame
        try:
            dx, dy, dz, drx, dry, drz = action
            self.robot.MoveLinRelTrf(dx, dy, dz, drx, dry, drz)
            
            # Wait for the relative move to complete before returning
            self.robot.WaitIdle() 
            
        except robot.MecademicException as e:
            print(f"An error occurred during set_action: {e}")
        except ValueError:
            print(f"Error: Action must be a 6-element array/list. Received: {action}")

    def close(self):
        print("Disconnecting from hardware...")
        
        # Disconnect Meca500
        try:
            if self.robot.IsConnected():
                self.robot.DeactivateRobot()
                self.robot.Disconnect()
                print("Meca500 disconnected.")
        except robot.MecademicException as e:
            print(f"An error occurred during Meca500 deactivation: {e}")

        # Disconnect force sensor
        # TODO: Add your force sensor disconnection code
        if self.force_sensor:
            try:
                self.force_sensor.close()
                print("Force sensor disconnected.")
            except Exception as e:
                print(f"An error occurred during force sensor disconnection: {e}")