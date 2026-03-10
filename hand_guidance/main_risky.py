import mecademicpy.robot as mdr
import bota_driver
import time
import json
from typing import Optional, Dict, Any
import numpy as np
import threading
    
class main():
    def __init__(self):
        self.sensor: bota_driver.BotaDriver | None = None
        self.robot = mdr.Robot()
        self.sensor_config_params = dict()
        self.sensor_config_params["update_rate"] = 800 # Hz, default value
        
        self.sensor_connected: bool = False
        self.meca_connected: bool = False

        self.meca_address = "192.168.0.100" # Default IP address for Mecademic robot

        self.json_path = "bota_sensor_config.json"
        self.sensor_type = "Bota_Binary_gen0"
        self.port = "COM4"  # Default port for serial connection
        
        self._running = False
        self._thread = None
        self.hg = None

        # Hand guidance parameters (Ported from hand_guidance.py)
        self.gain_tr = 10 #10
        self.gain_rot = 50  #50
        self.f_threshold_high = 0.5
        self.f_threshold_low = 0.1
        self.m_threshold_high = 0.05
        self.m_threshold_low = 0.01
        
        self.wrench_filter = np.zeros(6)
        self.alpha = 0.1 #  0.1 Simple low pass filter factor

    def start_robot(self):
        try:
            self.robot.Connect(address=self.meca_address)
            self.robot.ResetError()
            self.robot.ActivateAndHome()
            self.robot.WaitHomed()
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
    
    def _guidance_loop(self):
        """Runs the high-frequency force control loop."""
        #self.sensor.set_config_param("update_rate", 800)
        #self.sensor.set_config_param("filter_cutoff", 50)
        
        while self._running:
            # Read sensor
            frame_data = self.sensor.read_frame()
            if frame_data: # and frame_data.status == bota_driver.Status.VALID:
                wrench = np.array([
                    frame_data.force[0], frame_data.force[1], frame_data.force[2],
                    frame_data.torque[0], frame_data.torque[1], frame_data.torque[2]
                ])
                
                # Filter
                self.wrench_filter = (1 - self.alpha) * self.wrench_filter + self.alpha * wrench
                
                # Logic from script
                normF = np.linalg.norm(self.wrench_filter[:3])
                normM = np.linalg.norm(self.wrench_filter[3:])
                
                twist = np.zeros(6)
                
                # Translation
                if normF > self.f_threshold_high:
                     twist[:3] = self.gain_tr * self.wrench_filter[:3]
                
                # Rotation
                if normM > self.m_threshold_high:
                     twist[3:] = self.gain_rot * self.wrench_filter[3:]

                # Send Velocity to Robot
                # MoveLinVelWrf expects: x, y, z, wx, wy, wz
                self.robot.MoveLinVelTrf(
                    -twist[0], -twist[1], twist[2], 
                    -twist[3], -twist[4], twist[5]
                )
            
            time.sleep(0.002) # ~500Hz loop

    def start_hand_guidance(self):
        if self.meca_connected:
            self.robot.Disconnect()
            self.robot.WaitDisconnected()

        if not self.start_sensor():
            return False

        self.start_robot()
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

        try:
           self._running = True
           self._thread = threading.Thread(target=self._guidance_loop, daemon=True)
           self._thread.start()
        except Exception as e:
            print("Error", f"HandGuidance failed to start: {e}")
            return False
        return True
    
    def stop_hand_guidance(self):
            print("\nGracefully shutting down...")
            self._running = False

            if self._thread is not None:
                self._thread.join(timeout=1.0)
                self._thread = None

            
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
                    self.robot.Disconnect()
                    print("  Waiting for robot to confirm disconnection (this may take a moment)...")
                    self.robot.WaitDisconnected()
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
