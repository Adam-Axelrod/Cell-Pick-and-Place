import mecademicpy.robot as meca_robot
import bota_driver
import numpy as np

from .config_meca500_custom import Meca500CustomConfig


class Meca500Custom:
    """Meca500 robot with force sensor for LeRobot."""
    
    def __init__(self, config: Meca500CustomConfig | None = None, **kwargs):
        if config is None:
            config = Meca500CustomConfig()
        
        # Allow kwargs to override config
        for key, value in kwargs.items():
            if hasattr(config, key):
                setattr(config, key, value)
        
        self.config = config
        self.robot = None
        self.force_sensor = None
        
        print(f"Connecting to Meca500 at {self.config.ip_address}...")
        self._connect_hardware()

    def _connect_hardware(self):
        """Connect to robot and force sensor."""
        # Connect to Meca500
        self.robot = meca_robot.Robot()
        
        try:
            self.robot.Connect(address=self.config.ip_address)
            self.robot.ActivateRobot()
            self.robot.Home()
            self.robot.WaitHomed()
            self.robot.SetJointVel(5)
            print("Meca500 connected and homed.")
        except meca_robot.MecademicException as e:
            print(f"Error connecting to Meca500: {e}")
            raise

        # Connect to force sensor
        print(f"Connecting to force sensor at {self.config.sensor_config_path}...")
        try:
            self.force_sensor = bota_driver.BotaDriver(self.config.sensor_config_path)
            print("Force sensor connected.")
        except Exception as e:
            print(f"Warning: Could not connect to force sensor: {e}")
            self.force_sensor = None

    @property
    def cameras(self):
        """Return camera configuration."""
        return self.config.cameras

    def get_state(self):
        """Get current robot state (joint positions + force data)."""
        # Get joint angles
        joint_positions = self.robot.GetRtTargetJointPos()
        
        # Get force data
        force_data = [0.0] * 6
        if self.force_sensor:
            try:
                data = self.force_sensor.read_frame()
                force_data[0:3] = data.force
                force_data[3:6] = data.torque
            except Exception as e:
                print(f"Warning: Could not read force sensor: {e}")
        
        return {
            "joint_positions": list(joint_positions),
            "force": force_data
        }

    def send_action(self, action):
        """Send action to robot (relative Cartesian move)."""
        try:
            dx, dy, dz, drx, dry, drz = action
            self.robot.MoveLinRelTrf(dx, dy, dz, drx, dry, drz)
            self.robot.WaitIdle()
        except meca_robot.MecademicException as e:
            print(f"Error during action: {e}")
        except (ValueError, TypeError) as e:
            print(f"Invalid action format: {e}")

    def disconnect(self):
        """Disconnect from hardware."""
        print("Disconnecting from hardware...")
        
        if self.robot and self.robot.IsConnected():
            try:
                self.robot.Disconnect()
                self.robot.WaitDisconnected()
                print("Meca500 disconnected.")
            except Exception as e:
                print(f"Error disconnecting Meca500: {e}")
        
        if self.force_sensor:
            try:
                self.force_sensor.deactivate()
                self.force_sensor.shutdown()
                print("Force sensor disconnected.")
            except Exception as e:
                print(f"Error disconnecting force sensor: {e}")

    def __del__(self):
        """Cleanup on deletion."""
        self.disconnect()