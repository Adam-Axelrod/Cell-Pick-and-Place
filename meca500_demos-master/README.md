# Meca500 Demos
This is a package containing a few examples to demonstrate how to run a Meca500 TCP/IP configuration with either the Serial or EtherCAT Bota Systems force-torque sensor.

## Dependencies

### Operating system based dependencies
#### Linux

##### Setup Python environment 
When working with Python, it is recommended to create an isolated virtual environment. You can install the utility 'venv' with the following command:

```bash
sudo apt install python3-venv
```
We have setup a bash file to easily create virtual environments. 
If you have more than one python version installed in your system, it will ask with which one you would like to proceed.

```bash
./scripts/ubuntu/01_create_venv.sh 
```

If you prefer to do it manually, you can call this command:

```bash
python<VERSION> -m venv <VENV_DIRECTORY>

# `<VERSION>` is the version of Python you want to use (e.g. 3.8, 3.9, etc.) 
# `<VENV_DIRECTORY>` is the directory where you want to create the virtual environment.
```

**ADDITIONAL: Bota Serial Binary** 

Only if you are working with a Bota Binary communication interfaces, both Gen0 and GenA, you will need to make sure the user running the Python scripts belong to the dialout group. 

We have setup a bash file to easily add the user to the dialout group. You can run it with the following command:

```bash
sudo ./scripts/ubuntu/01b_add_user_to_dialout.sh 
```

Alternatively, you can do it manually with the following command:

```bash
sudo usermod -a -G dialout $(whoami)
```

After running the script/command, you will need to log out and log in again for the changes to take effect.

**ADDITIONAL: CANOpen over EtherCAT** 

Only if you are working with a CANopen over EtherCAT sensor, both Gen0 and GenA, you will need to setup special capabilities to access the network interface hardware for the executable, in this case the Python interpreter. The best way to do this is to subtitute the symlink to the system-wide Python interpreter in the virtual environemnt with a local copy, and then assign to this local copy of the python interpreter the correct capabilities. 

We have setup a bash file to easily do this. You can run it with the following command:

```bash
sudo ./scripts/ubuntu/01b_set_network_capabilities.sh 
```

Alternatively, if you do not mind running the python scripts with sudo rights, you can skip this step and run the scripts with sudo.


#### Windows

* [Python 3 / 64 Bit](https://www.python.org/downloads/) or through Microsoft Store
* [Npcap](https://nmap.org/npcap/)* or [WinPcap](https://www.winpcap.org/)

[*] Make sure you check "Install Npcap in WinPcap API-compatible Mode" during the installation.

## Installation

To install the dependencies required by the package, use the following command,
```bash
# On Windows
pip install -r requirements.txt

# On Linux
pip install -r requirements.txt
```
**NOTE**: For Serial sensors, it is recommended to change the latency settings of the sensor port. You can take the following steps to ensure that there is minimal latency in the sensor communication.

*On Windows*
1. Open Device Manager
2. Open Ports -> Your Sensor's Port -> Properties -> Port Settings -> Advanced -> Latency Timer
3. Select latency to be minimum value (1 msec)

*On Linux*
```bash
setserial <your port name> low_latency
```


## Examples

The examples demonstrate some of the tasks that can be achieved with force torque sensing feedback.

### 1. Hand Guidance

#### Description

Enabling guidance of robot in the tool frame based on contact forces.

1. When the demo file is launched, it connects with the Meca robot and the F/T sensor and activates both of them.
2. After the robot is activated, we perform a homing operation if it has not been done since the most recent power cycle, after which the robot goes to a default pose.
3. Once the robot is at the default pose, the sensor readings are zeroed out. **NOTE**: Do not touch the sensor for a few seconds after the robot reaches the default pose.
4. After a few seconds of waiting, you can move the robot around by pulling on the sensor.

#### Parameters
| Parameter Name     | Default Value | Description                                                                   |
|--------------------|:-------------:|-------------------------------------------------------------------------------|
| `f_threshold_high` |     0.20      | Threshold force norm needs to cross for robot to react to it.                 |
| `f_threshold_low`  |     0.05      | If force norm is less than this threshold, it stops reacting to force.        |
| `m_threshold_high` |     0.40      | Threshold moment norm needs to cross for robot to react to it.                |
| `m_threshold_low`  |     0.10      | If moment norm is less than this threshold, it stops reacting to momentforce. |
| `gain_tr`          |     10.0      | Proportional gain for translation                                             |
| `gain_rot`         |     50.0      | Proportional gain for rotation                                                |
| `alpha`            |     1.00      | Low pass filter parameter                                                     |

### 2. Button Testing

#### Description

Example of product testing by detecting contact and plotting the force profile during a defined button pressing motion.

1. When the demo file is launched, it connects with the Meca robot and the F/T sensor and activates both of them.
2. After the robot is activated, we perform a homing operation if it has not been done since the most recent power cycle, after which the robot goes to the start pose.
3. Once the robot is at the start pose, the sensor readings are zeroed out. **NOTE**: Do not touch the sensor for a few seconds after the robot reaches the default pose.
4. After a few seconds of waiting, robot moves in the requested direction until it makes contact with a surface and presses the button until the force threshold is reached.
5. Motion is reverted (releasing of button) until the retract distance is reached and repeated for the next pose

#### Parameters
| Parameter Name     | Default Value | Description                                                                                            |
|--------------------|:-------------:|--------------------------------------------------------------------------------------------------------|
| `force_threshold`  |     1.00      | Robot registers end stop contact when force norm crosses this threshold (in N).                                 |
| `search_direction` |    [0,0,-1]    | Direction in which robot moves until contact.                                                          |
| `reference_frame`  |     "TRF"     | Reference frame for direction vector (can be "WRF" or "TRF", world frame and tool frame respectively). |
| `velocity`         |     3.00      | Velocity with which robot moves in requested direction (in mm/s).                                      |
| `max_distance`     |     -10.0     | Maximum distance robot moves (in mm).                                                                  |
| `retract_distance` |     5.00      | Once threshold is reached, robot moves back by this distance (in mm).                                  |
| `start_pose` |     [182, -68, 105, 180, 0, -90]      | Initial pose (in mm & deg) for searching contact, afterwards robot is moving to poses relative to this one.  |

### 3. Contour Following

#### Description

Tracking of constant contact force in z-direction while moving in straight line back and forth in the Y-direction.

1. When the demo file is launched, it connects with the Meca robot and the F/T sensor and activates both of them.
2. After the robot is activated, we perform a homing operation if it has not been done since the most recent power cycle, after which the robot goes to a start pose.
3. Once the robot is at the start pose, the sensor readings are zeroed out. **NOTE**: Do not touch the sensor for a few seconds after the robot reaches the default pose.
4. After a few seconds of waiting, robot moves in the requested direction until it makes contact with a surface.
5. Once contact is registered, the robot moves along the surface in the Y-direction, while keeping the force in Z-direction constant.

#### Parameters (can be found as class variables in `contour_following_ethercat.py` and `contour_following_serial.py`)
| Parameter Name           |      Default Value       | Description                                                                            |
|--------------------------|:------------------------:|----------------------------------------------------------------------------------------|
| `start_pose`             | [170, -50, 180, 180, 0, -90] | Starting pose of the application (in mm & deg).                                        |
| `max_distance`           |           50.0           | Distance the End Effector moves in the Y-direction until it changes direction (in mm). |
| `force_threshold`        |           1.0            | Robot registers contact when force Fz crosses this threshold (in N).          |
| `force_setpoint`         |           -3.0            | Reference force to be tracked, when in contact (in N).                                
| `approach_velocity`      |          -5.0           | Velocity with which robot moves in Z-direction until contact is registered (in mm/s).  |
| `perpendicular_velocity` |           5.0            | Velocity with which robot moves in Y-direction after contact is registered (in mm/s).  |
| `PID parameters`         |     [8.0, 0.01, 0.1]      | PID parameters, Kp, Ki, Kd.                                                            |

### 4. Pick and Weigh

#### Description

Pick object with gripper. The sensor is tared before grasping the object and the weight difference is computed. Note that this example is not run as separate task but as sequential program.

1. When the demo file is launched, it connects with the Meca robot and the F/T sensor and activates both of them.
2. After the robot is activated, we perform a homing operation if it has not been done since the most recent power cycle, after which the robot goes to a start pose.
3. Once the robot is at the start pose, the sensor readings are zeroed out. **NOTE**: Do not touch the sensor for a few seconds after the robot reaches the start pose.
4. After a few seconds of waiting, the robot moves to the grasp pose to grasp the object and retracts after closing the gripper
5. Once the robot is retracted, the (average) force difference is measured to compute the weight of the grasped object. A message window is opened to display the weight.
6. After acknowledging the message, the object is placed again.

### 3. Contour Following

#### Description

1. When the demo file is launched, it connects with the Meca robot and the F/T sensor and activates both of them.
2. After the robot is activated, we perform a homing operation if it has not been done since the most recent power cycle, after which the robot goes to a default pose.
3. Once the robot is at the default pose, the sensor readings are zeroed out. **NOTE**: Do not touch the sensor for a few seconds after the robot reaches the default pose.
4. After a few seconds of waiting, robot moves in the requested direction until it makes contact with a surface.
5. Once contact is registered, the robot moves along the surface in the Y-direction, while keeping the force in Z-direction constant.

#### Parameters (can be found as class variables in `contour_following_ethercat.py` and `contour_following_serial.py`)
| Parameter Name           |      Default Value       | Description                                                                            |
|--------------------------|:------------------------:|----------------------------------------------------------------------------------------|
| `start_pose`             | [170, 0, 180, 180, 0, 0] | Starting pose of the application (in mm & deg).                                        |
| `max_distance`           |           50.0           | Distance the End Effector moves in the Y-direction until it changes direction (in mm). |
| `force_threshold`        |           1.0            | Robot registers contact when force Fz crosses this threshold (in N).                   |
| `set_force`              |           3.0            | Reference force to be tracked, when in contact (in N).                                 |
| `approach_velocity`      |          -10.0           | Velocity with which robot moves in Z-direction until contact is registered (in mm/s).  |
| `perpendicular_velocity` |           5.0            | Velocity with which robot moves in Y-direction after contact is registered (in mm/s).  |
| `PID parameters`         |     [4.0, 0.0, 0.1]      | PID parameters, Kp, Ki, Kd.                                                            |

## Usage
You can run the examples either through the GUI application (recommended), or directly through the command line (deprecated).

### GUI
To run the GUI application, run the following command
```bash
# On Windows
python gui/main.py

# On Linux
sudo su
python gui/main.py
```
This should launch the application window.

![gui-screenshot](gui/images/docs/meca500_gui.png)

### Utilities
To list out all available ports and finding the sensor port use the utility functions as follows,
```bash
# For EtherCAT sensors
python utils/find_ethernet_adapters.py

# For Serial sensors
python utils/find_serial_ports.py
```
NOTE: On Linux systems, run EtherCAT scripts with administrator privileges.

Serial ports may look like `COM5` on Windows and like `dev/ttyUSB0` on Linux.
Ethernet ports may look like `\Device\NPF_{XXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX}` on Windows, and like `enp0s31f6` on Linux.

## License
BSD3. See [LICENSE](LICENSE) file.
