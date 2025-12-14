import mecademicpy.robot as mdr
import bota_driver
import time
import json
from typing import Dict, Any
import hand_guidance

SERIALV0 = "Bota_Binary_gen0"
    
class main():
    def __init__(self):
        self.sensor: bota_driver.BotaDriver | None = None
        self.meca_robot = mdr.Robot()
        self.json_path = "bota_sensor_config.json"
        self.sensor_config_params = dict()
        self.sensor_config_params["update_rate"] = 800 # Hz, default value
        
        self.sensor_connected: bool = False
        self.meca_connected: bool = False

        self.sensor_type = SERIALV0
        self.port = "COM4"  # Default port for serial connection
        self.meca_address = "192.168.0.100" # Default IP address for Mecademic robot

        self.hg = None

    def start_robot(self):
        try:
            self.meca_robot.Connect(address=self.meca_address)
            self.meca_robot.ResetError()
            self.meca_robot.ActivateAndHome()
            self.meca_robot.WaitHomed()
            self.meca_connected = True
            print("Robot Activated")
        except mdr.MecademicException as e:
            print(f"Error connecting to robot: {e}")
            return
        
    def read_json(self, path: str) -> Dict[str, Any]:
        with open(path, "r", encoding="utf-8") as fh:
            return json.load(fh)

    def start_sensor(self):
        if self.sensor_connected:
            print("Warning", "Sensor already running! Stop sensor first to restart.")
            return False

        self.sensor = None
        try:
            # Load sensor config
            config_content = self.read_json(self.json_path)
            self.sensor = bota_driver.BotaDriver(self.json_path)

            # Transition driver from UNCONFIGURED to INACTIVE state
            if not self.sensor.configure():
                raise RuntimeError("Failed to configure driver")

            # Tare the sensor
            if not self.sensor.tare():
                raise RuntimeError("Failed to tare sensor")

            # Transition driver from INACTIVE to ACTIVE state
            if not self.sensor.activate():
                raise RuntimeError("Failed to activate driver")
        except Exception as e:
            print("Error", f"Something went wrong while activating Bota Serial Sensor: {e}")
            return False
        self.sensor_connected = True
        
        return True

    def start_hand_guidance(self):
        #if self.meca_connected:
        #    self.meca_robot.Disconnect()
        #    self.meca_robot.WaitDisconnected()

        if not self.start_sensor():
            return False
        
        if not self.meca_connected:
            try:
                self.start_robot()
            except Exception as e:
                print("Error", f"Failed to start robot: {e}")
                return False
        
        if not self.meca_connected:
            return False
        # pass sampling_period (seconds) computed from the configured update rate if available
        sampling_period = None
        try:
            ur_temp = float(self.sensor_config_params.get("update_rate", 0))
            if ur_temp > 0:
                sampling_period = 1.0 / ur_temp
        except Exception:
            sampling_period = None

        self.hg = hand_guidance.HandGuidance(self.sensor, self.meca_robot, sampling_period)

 
        if not self.hg.initialize():
            print("Error", "HandGuidance failed to initialize")
            self.hg = None
            self.stop_hand_guidance()
            return False

        try:
            self.hg.run()
        except Exception as e:
            print("Error", f"HandGuidance failed to start: {e}")
            self.hg = None
            return False
        return True
    
    def stop_hand_guidance(self):
            print("\nGracefully shutting down...")
            if self.hg is not None:
                print("  Stopping hand guidance module...")
                self.hg.stop()
                self.hg = None
            
            if self.sensor_connected and self.sensor is not None:
                print("  Deactivating and shutting down sensor...")
                try:
                    self.sensor.deactivate()
                    self.sensor.shutdown()
                except Exception as e:
                    print(f"  Error stopping sensor: {e}")
                self.sensor_connected = False
                self.sensor = None
            
            if self.meca_connected:
                print("  Disconnecting from robot...")
                try:
                    self.meca_robot.Disconnect()
                    print("  Waiting for robot to confirm disconnection (this may take a moment)...")
                    self.meca_robot.WaitDisconnected()
                    print("  Robot disconnected.")
                except mdr.MecademicException as e:
                    print(f"  Error disconnecting robot: {e}")
                self.meca_connected = False
            
            print("Shutdown complete. Exiting.")


if __name__ == "__main__":
    main_ = main()
    try:
        # Check if hand guidance started successfully
        if main_.start_hand_guidance():
            print("\nHand guidance is running. Press Ctrl+C to stop.")
            # Keep the main thread alive to catch the KeyboardInterrupt
            while True:
                time.sleep(1)
        else:
            print("Failed to start hand guidance.")
    except KeyboardInterrupt:
        # This block will now be triggered correctly
        print("\nCtrl+C detected! Stopping...")
        main_.stop_hand_guidance()
