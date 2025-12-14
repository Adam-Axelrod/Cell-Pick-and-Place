import hand_guidance
from libraries.meca import robot as rb
import bota_driver
import time


from utils.write_sensor_config import write_config
from utils.write_sensor_config import get_update_rate_gen_a
from utils.find_ethernet_adapters import find_adapters

SERIALV0 = "Bota_Binary_gen0"
USB_SERIAL = "Bota_Binary"
ETHERNET = "Bota_Socket"
ETHERCAT = "CANopen_over_EtherCAT"
ETHERCATV0 = "CANopen_over_EtherCAT_gen0"
    
class main():
    def __init__(self):
        self.sensor: bota_driver.BotaDriver | None = None
        self.meca_robot = rb.Robot()
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
        except rb.MecademicException as e:
            print(f"Error connecting to robot: {e}")
            return
        

    def start_sensor(self):
        if self.sensor_connected:
            print("Warning", "Sensor already running! Stop sensor first to restart.")
            return False
        try:
            comm_type = self.sensor_type
            selected_port = self.port
            update_rate = self.sensor_config_params["update_rate"]
        except KeyError:
            print("Error", "Select valid protocol and port!")
            return False

        self.sensor = None
        try:
            # Convert update rate to fit App submode
            if comm_type == ETHERCAT or comm_type == ETHERNET or comm_type == USB_SERIAL:
                update_rate = get_update_rate_gen_a(update_rate)
                self.sensor_config_params["update_rate"] = int(update_rate)

            if comm_type == ETHERCAT:
                self.sensor_config_path = write_config(protocol=comm_type, network_interface=selected_port, update_rate=update_rate)
            elif comm_type == ETHERCATV0:
                self.sensor_config_path = write_config(protocol=comm_type, network_interface=selected_port, update_rate=update_rate)
            elif comm_type == ETHERNET:
                self.sensor_config_path = write_config(protocol=comm_type, network_interface=selected_port, update_rate=update_rate)
            elif comm_type == SERIALV0:
                self.sensor_config_path = write_config(protocol=comm_type, com_port=selected_port, update_rate=update_rate)
            elif comm_type == USB_SERIAL:
                self.sensor_config_path = write_config(protocol=comm_type, com_port=selected_port, update_rate=update_rate)
            else:
                print("Error", "Comm type not supported!")
                return False
            #self.sensor = bota_driver.BotaDriver(self.sensor_config_path)
            self.sensor = bota_driver.BotaDriver("C:/Users/aa24/PhD/Cell-Pick-and-Place/lerobot_robot_meca500/lerobot_robot_meca500/sensor_config.json")

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
        if self.meca_connected:
            self.meca_robot.Disconnect()
            self.meca_robot.WaitDisconnected()

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

        self.hg = hand_guidance.HandGuidance(self.sensor, self.meca_robot, sampling_period)

 
        if not self.hg.initialize():
            print("Error", "HandGuidance failed to initialize")
            self.hg = None
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
                except rb.MecademicException as e:
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
