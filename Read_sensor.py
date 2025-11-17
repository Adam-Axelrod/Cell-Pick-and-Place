import time
from bota_driver import BotaDriver, BotaFrame, SensorConnectionError

# Initialize the driver object
driver = None

try:
    print("Connecting to sensor...")
    # 1. Initialize the driver (auto-detects USB)
    driver = BotaDriver()
    
    # 2. Configure the driver (loads settings)
    driver.configure()
    
    # 3. Activate the data stream
    driver.activate()

    print("Sensor connected. Activating stream.")
    print("Press Ctrl+C to stop.\n")

    # 4. Tare (zero) the sensor
    # It's good practice to do this before reading
    print("Taring sensor... keep it still.")
    driver.tare()
    time.sleep(1) # Give it a moment

    # 5. Read data in a loop
    while True:
        # read_frame_blocking() waits for the next new data frame
        frame = driver.read_frame_blocking()

        # Access the force (f) and torque (t) data
        print(
            f"FX: {frame.fx:6.2f} N,  "
            f"FY: {frame.fy:6.2f} N,  "
            f"FZ: {frame.fz:6.2f} N,  "
            f"TX: {frame.tx:6.2f} Nm, "
            f"TY: {frame.ty:6.2f} Nm, "
            f"TZ: {frame.tz:6.2f} Nm",
            end='\r' # Print on the same line
        )

except SensorConnectionError as e:
    print(f"Error: Could not connect to sensor. {e}")
    print("Make sure the sensor is plugged in and not in use by another program.")

except KeyboardInterrupt:
    print("\nStopping data stream.")

finally:
    # 6. CRITICAL: Deactivate and cleanup
    if driver:
        print("\nDeactivating and cleaning up...")
        driver.deactivate()
        driver.cleanup()
        print("Done.")