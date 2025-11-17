import sys
import os
import time

package_path = os.path.dirname(os.path.dirname(__file__))
sys.path.append(package_path)

from typing import Union, Optional, Type

import tkinter as tk
from tkinter import ttk, font, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from PIL import Image, ImageTk
import serial.tools.list_ports

import numpy as np

from gui.plotter import Plotter

import bota_driver

from libraries.meca import robot_common, robot as rb

from tasks.task import Task
from tasks.hand_guidance import HandGuidance
from tasks.button_testing import ButtonTesting
from tasks.deflashing import Deflashing
from tasks.alignment import Alignment
from tasks.polishing import Polishing
from tasks.part_polishing import PartPolishing
from tasks.contour_following import ContourFollowing
from utils.write_sensor_config import write_config
from utils.write_sensor_config import get_update_rate_gen_a
from utils.find_ethernet_adapters import find_adapters

SERIALV0 = "Bota_Binary_gen0"
USB_SERIAL = "Bota_Binary"
ETHERNET = "Bota_Socket"
ETHERCAT = "CANopen_over_EtherCAT"
ETHERCATV0 = "CANopen_over_EtherCAT_gen0"


class BotaWindow(tk.Tk):
    def __init__(self):
        super().__init__()

        self.wm_title("Bota Systems: Meca500 Demos")
        self.geometry("1920x1080")
        self.resizable(True, True)

        self.plotter = Plotter()
        self.task: Optional[Task] = None
        self.sensor: bota_driver.BotaDriver | None = None

        self.sensor_type = tk.StringVar()
        self.sensor_port_desc = tk.StringVar()
        self.ports = dict()
        self.sensor_config_params = dict()
        self.sensor_config_params["update_rate"] = 800 # Hz, default value
        self.sensor_config_path = os.path.join(os.path.dirname(__file__), "gui/bft_config", "sensor_config.json")

        self.meca_robot = rb.Robot()
        self.meca_ip_entry: Optional[tk.Entry] = None

        self.sensor_connected: bool = False
        self.sensor_status_light_canvas: Optional[tk.Canvas] = None
        self.sensor_status_light_oval = None
        self.sensor_status_label_text = tk.StringVar()
        self.meca_connected: bool = False
        self.meca_status_light_canvas: Optional[tk.Canvas] = None
        self.meca_status_light_oval = None
        self.meca_status_label_text = tk.StringVar()
        self.frequency_label_text = tk.StringVar()

        self.style = ttk.Style()
        self.style.configure('.', font=('Helvetica', 12))
        bigfont = font.Font(family="Helvetica", size=12)
        self.option_add("*Font", bigfont)

        bota_logo_path = os.path.join(os.path.dirname(__file__), "images", "Bota-LogoFull-Black.png")
        bota_logo_image = Image.open(bota_logo_path)
        new_width = 125
        bota_logo_image = bota_logo_image.resize(
            (new_width, int(bota_logo_image.size[1] * new_width / bota_logo_image.size[0])))
        self.bota_logo = ImageTk.PhotoImage(bota_logo_image)

        bota_icon_path = os.path.join(os.path.dirname(__file__), "images", "Bota-LogoSquare-Black.png")
        bota_icon_image = Image.open(bota_icon_path)
        new_width = 25
        bota_icon_image = bota_icon_image.resize(
            (new_width, int(bota_icon_image.size[1] * new_width / bota_icon_image.size[0])))
        self.bota_icon = ImageTk.PhotoImage(bota_icon_image)

        self.protocol("WM_DELETE_WINDOW", self.stop_sequence)
        self.iconphoto(False, self.bota_icon)
        self.organize_widgets()

    def stop_sequence(self):
        if self.sensor:
            self.sensor.shutdown()
        if self.task:
            self.task.stop()
        if self.meca_connected:
            self.meca_robot.Disconnect()
            self.meca_robot.WaitDisconnected()
        self.plotter.stop()
        self.quit()

    def update_sensor_connected(self, connected: bool):
        self.sensor_connected = connected
        if self.sensor_connected:
            self.sensor_status_light_canvas.itemconfig(self.sensor_status_light_oval, fill="green")
            self.sensor_status_label_text.set("Sensor Connected")
            # self.frequency_label_text.set("Sampling Rate: {:0.2f} Hz".format(1. / self.sensor.get_expected_timestep()))
            self.frequency_label_text.set("Sampling Rate: {:0.2f} Hz".format(self.sensor_config_params["update_rate"]))
        else:
            self.sensor_status_light_canvas.itemconfig(self.sensor_status_light_oval, fill="red")
            self.sensor_status_label_text.set("Sensor Not Connected")
            self.frequency_label_text.set("Sampling Rate: --")

    def update_meca_connected(self, connected: bool):
        self.meca_connected = connected
        if self.meca_connected:
            self.meca_status_light_canvas.itemconfig(self.meca_status_light_oval, fill="green")
            self.meca_status_label_text.set("Meca500 Connected")
        else:
            self.meca_status_light_canvas.itemconfig(self.meca_status_light_oval, fill="red")
            self.meca_status_label_text.set("Meca500 Not Connected")

    def kill_tasks(self):
        self.plotter.stop()
        if self.task:
            self.task.stop()
        self.stop_sensor()
        if self.meca_connected:
            self.meca_robot.ClearMotion()
            self.meca_robot.ResetError()
            self.meca_robot.ResumeMotion()
            self.meca_robot.WaitIdle()

    def kill_tasks_go_home(self):
        self.plotter.stop()
        if self.task:
            self.task.stop()
        self.stop_sensor()
        if self.meca_connected:
            self.meca_robot.ClearMotion()
            self.meca_robot.ResetError()
            self.meca_robot.ResumeMotion()

            self.meca_robot.WaitIdle()
            self.meca_robot.MoveLinRelWRF(0, 0, 30, 0, 0, 0)
            self.meca_robot.WaitIdle()

            self.meca_robot.MovePose(170, -50, 180, 180, 0, -90)
            self.meca_robot.WaitIdle()

    def add_task(self, task: Type[Task]) -> bool:
        self.kill_tasks()
        if self.meca_connected:
            self.meca_robot.Disconnect()
            self.meca_robot.WaitDisconnected()
        self.update_meca_connected(False)
        self.update_sensor_connected(False)

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

        self.task = task(self.sensor, self.meca_robot, sampling_period)

        if self.task.initialize():
            self.update_meca_connected(True)
            self.meca_robot.WaitHomed()

            return True
        else:
            return False

    def run_task(self):
        # self.plotter.run()
        if self.task:
            self.task.run()

    def start_sensor(self):
        if self.sensor_connected:
            messagebox.showwarning("Warning", "Sensor already running! Stop sensor first to restart.")
            return False
        try:
            comm_type = self.sensor_type.get()
            selected_port = self.ports[self.sensor_port_desc.get()]
            update_rate = self.sensor_config_params["update_rate"]
        except KeyError:
            messagebox.showerror("Error", "Select valid protocol and port!")
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
                messagebox.showerror("Error", "Comm type not supported!")
                return False
            self.sensor = bota_driver.BotaDriver(self.sensor_config_path)

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
            messagebox.showerror("Error", f"Something went wrong while activating Bota Serial Sensor: {e}")
            return False

        self.update_sensor_connected(True)
        self.plotter.add_sensor(self.sensor)
        self.plotter.run()
        return True

    def stop_sensor(self):
        self.plotter.stop()
        if self.sensor:
            self.sensor.deactivate()
            self.sensor.shutdown()
            print("Sensor Deactivated")

        self.update_sensor_connected(False)
        self.sensor = None
        return True

    def start_robot(self):
        self.meca_robot.Connect(address=self.meca_ip_entry.get())
        self.meca_robot.ResetError()
        self.meca_robot.ActivateAndHome()
        self.meca_robot.WaitHomed()
        self.update_meca_connected(True)
        self.meca_connected = True
        print("Robot Activated")

    def stop_robot(self):
        self.meca_robot.Disconnect()
        self.meca_robot.WaitDisconnected()
        self.update_meca_connected(False)
        self.meca_connected = False
        print("Robot Disconnected")

    def set_robot_motion_speed(self):
        if self.meca_connected:
            self.meca_robot.SetJointAcc(20)
            self.meca_robot.SetCartAcc(30)
            self.meca_robot.SetJointVel(20)
            self.meca_robot.SetJointAcc(50)
            self.meca_robot.SetVelTimeout(0.01)

    def create_bota_logo_frame(self, root: Union[ttk.Frame, tk.Tk]) -> ttk.Frame:
        frame = ttk.Frame(root)
        logo_label = tk.Label(frame, image=self.bota_logo)
        logo_label.pack(side=tk.TOP, fill=tk.X, expand=True, pady=10)

        return frame

    def create_plotter_frame(self, root: Union[ttk.Frame, tk.Tk]) -> ttk.Frame:
        frame = ttk.Frame(root)

        # Make the frame expandable
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

        canvas = FigureCanvasTkAgg(self.plotter.fig, master=frame)
        canvas.draw()

        # Store canvas reference in the plotter
        self.plotter.canvas = canvas

        # Pack canvas with fill and expand options
        canvas_widget = canvas.get_tk_widget()
        canvas_widget.grid(row=0, column=0, sticky="nsew")

        # Add toolbar
        from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk
        toolbar_frame = ttk.Frame(frame)
        toolbar_frame.grid(row=1, column=0, sticky="ew")
        toolbar = NavigationToolbar2Tk(canvas, toolbar_frame)
        toolbar.update()

        # Define minimum size for the figure
        MIN_WIDTH = 400  # pixels
        MIN_HEIGHT = 300  # pixels

        # Ensure the animation and manual resizing don't conflict
        def on_resize(event):
            # Only process if the event is from the canvas widget itself
            if event.widget == canvas_widget and event.width > 1 and event.height > 1:
                # Get new dimensions, ensure they meet minimum sizes
                w = max(event.width, MIN_WIDTH)
                h = max(event.height, MIN_HEIGHT)

                # Temporarily stop animation to prevent redraw conflicts
                self.plotter._ani.event_source.stop()

                # Set the figure size in inches
                dpi = self.plotter.fig.dpi
                self.plotter.fig.set_size_inches(w / dpi, h / dpi, forward=True)

                # Make sure all elements fit properly
                self.plotter.fig.tight_layout()

                # Full redraw of the canvas
                canvas.draw()

                # Restart animation
                self.plotter._ani.event_source.start()

        # Connect the resize event to the canvas widget
        canvas_widget.bind("<Configure>", on_resize)

        # Set minimum size constraints on the canvas widget itself
        canvas_widget.config(width=MIN_WIDTH, height=MIN_HEIGHT)

        return frame

    def create_status_frame(self, root: Union[ttk.Frame, tk.Tk]) -> ttk.Frame:
        frame = ttk.Frame(root)

        def create_sensor_connected_frame() -> ttk.Frame:
            subframe = ttk.Frame(frame)

            self.sensor_status_light_canvas = tk.Canvas(subframe, width=20, height=20)
            self.sensor_status_light_oval = self.sensor_status_light_canvas.create_oval(2, 2, 18, 18, fill="red")
            self.sensor_status_light_canvas.pack(side=tk.LEFT, expand=True, padx=5, pady=5)

            label = ttk.Label(subframe, textvariable=self.sensor_status_label_text, font=("Helvetica", 10))
            self.sensor_status_label_text.set("Sensor Not Connected")
            label.pack(side=tk.LEFT, expand=True, padx=5, pady=5)

            return subframe

        sensor_connected_frame = create_sensor_connected_frame()
        sensor_connected_frame.pack(side=tk.LEFT, expand=True, padx=5, pady=5)

        def create_meca_connected_frame() -> ttk.Frame:
            subframe = ttk.Frame(frame)

            self.meca_status_light_canvas = tk.Canvas(subframe, width=20, height=20)
            self.meca_status_light_oval = self.meca_status_light_canvas.create_oval(2, 2, 18, 18, fill="red")
            self.meca_status_light_canvas.pack(side=tk.LEFT, expand=True, padx=5, pady=5)

            label = ttk.Label(subframe, textvariable=self.meca_status_label_text, font=("Helvetica", 10))
            self.meca_status_label_text.set("Meca500 Not Connected")
            label.pack(side=tk.LEFT, expand=True, padx=5, pady=5)

            return subframe

        meca_connected_frame = create_meca_connected_frame()
        meca_connected_frame.pack(side=tk.RIGHT, expand=True, padx=5, pady=5)

        return frame

    def create_sensor_config_frame(self, root: Union[ttk.Frame, tk.Tk]) -> ttk.Frame:
        frame = ttk.Frame(root)

        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(2, weight=1)

        main_label = ttk.Label(frame, text="Sensor Configuration", font=("Helvetica", 12, "bold"))
        main_label.grid(column=0, row=0, columnspan=3)

        frequency_label = ttk.Label(frame, textvariable=self.frequency_label_text, font=("Helvetica", 10))
        self.frequency_label_text.set("Sampling Rate: --")
        frequency_label.grid(column=2, row=0)

        sensor_types = [ETHERNET, ETHERCAT, ETHERCATV0, SERIALV0, USB_SERIAL]
        sensor_type_combobox = ttk.Combobox(frame, textvariable=self.sensor_type, values=sensor_types)
        sensor_type_combobox.set("Select sensor type")
        sensor_type_combobox.grid(column=0, row=1, columnspan=2, sticky=tk.W + tk.E, padx=5, pady=5)

        def config_sensor() -> tk.Toplevel:
            if not self.sensor_type.get() == SERIALV0 and not self.sensor_type.get() == ETHERCAT and not self.sensor_type.get() == ETHERCATV0 and not self.sensor_type.get() == USB_SERIAL and not self.sensor_type.get() == ETHERNET:
                messagebox.showwarning("Warning", "Choose a sensor type before!")
                return

            options_window = tk.Toplevel(self)
            options_window.grab_set()

            options_window.columnconfigure(0, weight=4)
            options_window.columnconfigure(1, weight=1)

            options_label = ttk.Label(options_window, text="Sensor Configuration Parameters",
                                      font=("Helvetica", 12, "bold"))
            options_label.grid(column=0, row=0, columnspan=2, padx=5, pady=5)

            def callback(P):
                if str(P).isnumeric():
                    return True
                return False

            vcmd = (options_window.register(callback))

            update_rate_label = ttk.Label(options_window, text="Update Rate [Hz]:")
            update_rate_label.grid(column=0, row=1, padx=5, pady=5, sticky=tk.W)
            update_rate_entry = ttk.Entry(options_window, validate='all', validatecommand=(vcmd, '%P'))
            update_rate_entry.insert(0, str(self.sensor_config_params["update_rate"]))
            update_rate_entry.grid(column=1, row=1, padx=5, pady=5, sticky=tk.W + tk.E)

            def update_params():
                # temp_sampling_rate = 1. / (0.00001953125 * float(str(update_rate_entry.get())))
                update_rate_entry_value = int(update_rate_entry.get())
                if (self.sensor_type.get() == SERIALV0 and not (50 <= update_rate_entry_value <= 800)):
                    messagebox.showwarning("Warning",
                                           "For Serial Gen0 sensors, ensure update rate is within 50 and 800.")
                    return
                elif (self.sensor_type.get() == ETHERCATV0 and not (150 <= update_rate_entry_value <= 2000)):
                    messagebox.showwarning("Warning",
                                           "For EtherCAT Gen0 sensors, ensure update rate is within 150 and 2000.")
                    return
                elif ((self.sensor_type.get() == ETHERCAT or self.sensor_type.get() == ETHERNET or self.sensor_type.get() == USB_SERIAL) and not (10 <= update_rate_entry_value <= 3840)):
                    messagebox.showwarning("Warning",
                                           "For GenA sensors, ensure update rate is within 10 and 3840.")

                    return

                self.sensor_config_params["update_rate"] = update_rate_entry_value
                options_window.destroy()

            run_button = ttk.Button(options_window, text="Set parameters", command=update_params)
            run_button.grid(column=0, row=2, columnspan=2, padx=5, pady=5)

            return options_window

        config_sensor_button = ttk.Button(frame, text="Config Sensor", command=config_sensor)
        config_sensor_button.grid(column=2, row=1, sticky=tk.W + tk.E, padx=5, pady=5)

        sensor_port_combobox = ttk.Combobox(frame, textvariable=self.sensor_port_desc, values=[])
        sensor_port_combobox.set("Select sensor port")
        sensor_port_combobox.grid(column=0, row=2, columnspan=2, sticky=tk.W + tk.E, padx=5, pady=5)

        def update_available_ports(*args):
            self.ports.clear()
            if self.sensor_type.get() == SERIALV0 or self.sensor_type.get() == USB_SERIAL:
                ports = serial.tools.list_ports.comports()
                for port in ports:
                    self.ports[port.description] = port.device
            elif self.sensor_type.get() == ETHERCAT or self.sensor_type.get() == ETHERCATV0 or self.sensor_type.get() == ETHERNET:
                ports = find_adapters()
                for port in ports:
                    # port may be a pysoem adapter or our Adapter namedtuple
                    name = getattr(port, "name", None)
                    desc = getattr(port, "desc", None)
                    # pysoem desc can be bytes; decode if needed
                    if isinstance(desc, (bytes, bytearray)):
                        try:
                            desc = str(desc, encoding="utf-8")
                        except Exception:
                            desc = str(desc)
                    if name is None:
                        continue
                    self.ports[str(desc)] = name
            else:
                messagebox.showwarning("Warning", "Select valid sensor type!")
                return

            sensor_port_combobox.set("Select sensor port")
            sensor_port_combobox.config(values=list(self.ports.keys()))

        sensor_type_combobox.bind("<<ComboboxSelected>>", update_available_ports)

        reload_ports_button = ttk.Button(frame, text="Update Ports", command=update_available_ports)
        reload_ports_button.grid(column=2, row=2, sticky=tk.W + tk.E, padx=5, pady=5)

        def start_sensor():
            self.start_sensor()
            self.update_sensor_connected(True)
            self.plotter.add_sensor(self.sensor)
            self.plotter.run()

        start_sensor_button = ttk.Button(frame, text="Start Sensor", command=start_sensor)
        start_sensor_button.grid(column=0, row=3, sticky=tk.W + tk.E, padx=5, pady=5)

        def zero_sensor():
            self.plotter.stop()
            if self.sensor:
                self.sensor.deactivate()
                self.sensor.tare()
                self.sensor.activate()
                self.plotter.add_sensor(self.sensor)
                self.plotter.run()
            else:
                messagebox.showwarning("Warning", "No active sensor available!")

        zero_sensor_button = ttk.Button(frame, text="Zero Sensor", command=zero_sensor)
        zero_sensor_button.grid(column=1, row=3, sticky=tk.W + tk.E, padx=5, pady=5)

        def stop_sensor():
            self.plotter.stop()
            self.stop_sensor()

        stop_sensor_button = ttk.Button(frame, text="Stop Sensor", command=stop_sensor)
        stop_sensor_button.grid(column=2, row=3, sticky=tk.W + tk.E, padx=5, pady=5)

        return frame

    def create_meca_config_frame(self, root: Union[ttk.Frame, tk.Tk]) -> ttk.Frame:
        frame = ttk.Frame(root)

        label = ttk.Label(frame, text="Meca500 Configuration", font=("Helvetica", 12, "bold"))
        label.pack(side=tk.TOP, expand=True, padx=5, pady=5)

        # label.grid(column=0, row=0, columnspan=3)

        def create_meca_ip_frame() -> ttk.Frame:
            subframe = ttk.Frame(frame)

            meca_ip_label = ttk.Label(subframe, text="Meca500 IP Address:")
            meca_ip_label.pack(side=tk.LEFT, fill=tk.X, padx=5, pady=5)

            self.meca_ip_entry = ttk.Entry(subframe)
            self.meca_ip_entry.insert(0, "192.168.1.100")
            self.meca_ip_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)

            return subframe

        meca_ip_frame = create_meca_ip_frame()
        meca_ip_frame.pack(side=tk.TOP, fill=tk.X, expand=True, padx=5, pady=5)

        def create_meca_control_panel_frame() -> ttk.Frame:
            subframe = ttk.Frame(frame)

            subframe.columnconfigure(0, weight=1)
            subframe.columnconfigure(1, weight=1)
            subframe.columnconfigure(2, weight=1)

            def activate_meca():
                try:
                    self.meca_robot.Connect(address=self.meca_ip_entry.get())
                    self.meca_robot.SetVelTimeout(0.10)

                except robot_common.CommunicationError:
                    messagebox.showerror("Error", "Something went wrong while activating the Meca500 robot!")
                    return

                self.update_meca_connected(True)

                self.meca_robot.ActivateAndHome()
                messagebox.showinfo("Info",
                                    "Wait for robot to comeplete homing sequence before sending other commands to the robot.")

                self.meca_robot.WaitHomed()

                if self.meca_connected:
                    self.meca_robot.ClearMotion()
                    self.meca_robot.ResetError()
                    self.meca_robot.ResumeMotion()
                    self.meca_robot.SetJointAcc(5)
                    self.meca_robot.SetJointVel(25)
                    self.meca_robot.MoveJoints(0, 30, -30, 0, 60, 0)
                    self.meca_robot.SetJointAcc(20)
                    self.meca_robot.SetCartAcc(50)
                    self.meca_robot.SetVelTimeout(0.01)
                else:
                    messagebox.showwarning("Warning", "No connected Meca500 found!")

                messagebox.showinfo("Info", "Meca500 sucessfully activated and homed.")

            activate_meca_button = ttk.Button(subframe, text="Activate", command=activate_meca)
            activate_meca_button.grid(column=0, row=0, sticky=tk.W + tk.E, padx=5, pady=5)

            def disconnect_robot():
                if self.meca_connected:
                    self.meca_robot.Disconnect()
                    self.meca_robot.WaitDisconnected()
                    self.update_meca_connected(False)
                else:
                    messagebox.showwarning("Warning", "No connected Meca500 found!")

            disconnect_robot_button = ttk.Button(subframe, text="Disconnect", command=disconnect_robot)
            disconnect_robot_button.grid(column=1, row=0, sticky=tk.W + tk.E, padx=5, pady=5)

            def move_to_home():
                self.kill_tasks()
                if self.meca_connected:
                    self.meca_robot.ClearMotion()
                    self.meca_robot.ResetError()
                    self.meca_robot.ResumeMotion()
                    self.meca_robot.SetJointAcc(30)
                    self.meca_robot.SetCartAcc(50)
                    self.meca_robot.SetJointVel(25)
                    self.meca_robot.MovePose(170, -50, 180, 180, 0, -90)
                    self.meca_robot.SetJointAcc(150)
                    self.meca_robot.SetCartAcc(600)
                    self.meca_robot.SetVelTimeout(0.01)
                else:
                    messagebox.showwarning("Warning", "No connected Meca500 found!")

            move_to_home_button = ttk.Button(subframe, text="Move to Home", command=move_to_home)
            move_to_home_button.grid(column=2, row=0, sticky=tk.W + tk.E, padx=5, pady=5)

            return subframe

        meca_control_panel_frame = create_meca_control_panel_frame()
        meca_control_panel_frame.pack(side=tk.TOP, fill=tk.X, expand=True)

        def create_gripper_control_panel_frame() -> ttk.Frame:
            subframe = ttk.Frame(frame)

            def open_gripper():
                self.kill_tasks()
                if self.meca_connected:
                    self.meca_robot.ClearMotion()
                    self.meca_robot.ResetError()
                    self.meca_robot.ResumeMotion()

                    self.meca_robot.WaitIdle()
                    self.meca_robot.GripperOpen()
                    self.meca_robot.WaitIdle()
                else:
                    messagebox.showwarning("Warning", "No connected Meca500 found!")

            open_button = ttk.Button(subframe, text="Open Gripper", command=open_gripper)
            open_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)

            def close_gripper():
                self.kill_tasks()
                if self.meca_connected:
                    self.meca_robot.ClearMotion()
                    self.meca_robot.ResetError()
                    self.meca_robot.ResumeMotion()

                    self.meca_robot.WaitIdle()
                    self.meca_robot.GripperClose()
                    self.meca_robot.WaitIdle()
                else:
                    messagebox.showwarning("Warning", "No connected Meca500 found!")

            close_button = ttk.Button(subframe, text="Close Gripper", command=close_gripper)
            close_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)

            return subframe

        gripper_control_panel_frame = create_gripper_control_panel_frame()
        gripper_control_panel_frame.pack(side=tk.TOP, fill=tk.X, expand=True)

        def create_tool_control_panel_frame() -> ttk.Frame:
            subframe = ttk.Frame(frame)

            def pick_tool_1():
                self.kill_tasks()
                if self.meca_connected:
                    self.meca_robot.ClearMotion()
                    self.meca_robot.ResetError()
                    self.meca_robot.ResumeMotion()

                    self.set_robot_motion_speed()

                    self.meca_robot.WaitIdle()
                    self.meca_robot.GripperOpen()
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.MovePose(183, 65, 96, 180, 0, -90)
                    self.meca_robot.MovePose(183, 86, 96, 180, 0, -90)
                    self.meca_robot.MoveLinRelTRF(0, 0, 10.5, 0, 0, 0)
                    self.meca_robot.GripperClose()
                    self.meca_robot.MoveLinRelTRF(-30, 0, 0, 0, 0, 0)
                    self.meca_robot.MoveLinRelTRF(0, 0, -20, 0, 0, 0)
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.WaitIdle()
                else:
                    messagebox.showwarning("Warning", "No connected Meca500 found!")

            pick_tool_1_button = ttk.Button(subframe, text="Pick T1", command=pick_tool_1)
            pick_tool_1_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)

            def place_tool_1():
                self.kill_tasks()
                if self.meca_connected:
                    self.meca_robot.ClearMotion()
                    self.meca_robot.ResetError()
                    self.meca_robot.ResumeMotion()

                    self.set_robot_motion_speed()

                    self.meca_robot.WaitIdle()
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.MovePose(183.5, 65, 85.5, 180, 0, -90)
                    self.meca_robot.MoveLinRelTRF(21, 0, 0, 0, 0, 0)
                    self.meca_robot.GripperOpen()
                    self.meca_robot.MoveLinRelTRF(0, 0, -20, 0, 0, 0)
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.WaitIdle()
                else:
                    messagebox.showwarning("Warning", "No connected Meca500 found!")

            place_tool_1_button = ttk.Button(subframe, text="Place T1", command=place_tool_1)
            place_tool_1_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)

            def pick_tool_2():
                self.kill_tasks()
                if self.meca_connected:
                    self.meca_robot.ClearMotion()
                    self.meca_robot.ResetError()
                    self.meca_robot.ResumeMotion()

                    self.set_robot_motion_speed()

                    self.meca_robot.WaitIdle()
                    self.meca_robot.GripperOpen()
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.MovePose(140, 65, 96, 180, 0, -90)
                    self.meca_robot.MovePose(140, 86, 96, 180, 0, -90)
                    self.meca_robot.MoveLinRelTRF(0, 0, 10.5, 0, 0, 0)
                    self.meca_robot.GripperClose()
                    self.meca_robot.MoveLinRelTRF(-30, 0, 0, 0, 0, 0)
                    self.meca_robot.MoveLinRelTRF(0, 0, -20, 0, 0, 0)
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.WaitIdle()
                else:
                    messagebox.showwarning("Warning", "No connected Meca500 found!")

            pick_tool_2_button = ttk.Button(subframe, text="Pick T2", command=pick_tool_2)
            pick_tool_2_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)

            def place_tool_2():
                self.kill_tasks()
                if self.meca_connected:
                    self.meca_robot.ClearMotion()
                    self.meca_robot.ResetError()
                    self.meca_robot.ResumeMotion()

                    self.set_robot_motion_speed()

                    self.meca_robot.WaitIdle()
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.MovePose(140, 65, 85.5, 180, 0, -90)
                    self.meca_robot.MoveLinRelTRF(21, 0, 0, 0, 0, 0)
                    self.meca_robot.GripperOpen()
                    self.meca_robot.MoveLinRelTRF(0, 0, -20, 0, 0, 0)
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.WaitIdle()
                else:
                    messagebox.showwarning("Warning", "No connected Meca500 found!")

            place_tool_2_button = ttk.Button(subframe, text="Place T2", command=place_tool_2)
            place_tool_2_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)

            def pick_tool_3():
                self.kill_tasks()
                if self.meca_connected:
                    self.meca_robot.ClearMotion()
                    self.meca_robot.ResetError()
                    self.meca_robot.ResumeMotion()

                    self.set_robot_motion_speed()

                    self.meca_robot.WaitIdle()
                    self.meca_robot.GripperOpen()
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.MovePose(95.5, 65, 96, 180, 0, -90)
                    self.meca_robot.MovePose(95.5, 86, 96, 180, 0, -90)
                    self.meca_robot.MoveLinRelTRF(0, 0, 10.5, 0, 0, 0)
                    self.meca_robot.GripperClose()
                    self.meca_robot.MoveLinRelTRF(-30, 0, 0, 0, 0, 0)
                    self.meca_robot.MoveLinRelTRF(0, 0, -20, 0, 0, 0)
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.WaitIdle()
                else:
                    messagebox.showwarning("Warning", "No connected Meca500 found!")

            pick_tool_3_button = ttk.Button(subframe, text="Pick T3", command=pick_tool_3)
            pick_tool_3_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)

            def place_tool_3():
                self.kill_tasks()
                if self.meca_connected:
                    self.meca_robot.ClearMotion()
                    self.meca_robot.ResetError()
                    self.meca_robot.ResumeMotion()

                    self.set_robot_motion_speed()

                    self.meca_robot.WaitIdle()
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.MovePose(95.5, 65, 100, 180, 0, -90)
                    self.meca_robot.MovePose(95.5, 65, 85.5, 180, 0, -90)
                    self.meca_robot.MoveLinRelTRF(21, 0, 0, 0, 0, 0)
                    self.meca_robot.GripperOpen()
                    self.meca_robot.MoveLinRelTRF(0, 0, -20, 0, 0, 0)
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.WaitIdle()
                else:
                    messagebox.showwarning("Warning", "No connected Meca500 found!")

            place_tool_3_button = ttk.Button(subframe, text="Place T3", command=place_tool_3)
            place_tool_3_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)
            return subframe

        tool_control_panel_frame = create_tool_control_panel_frame()
        tool_control_panel_frame.pack(side=tk.TOP, fill=tk.X, expand=True)

        return frame

    def create_task_selection_frame(self, root: Union[ttk.Frame, tk.Tk]) -> ttk.Frame:
        frame = ttk.Frame(root)

        label = ttk.Label(frame, text="Example Tasks", font=("Helvetica", 12, "bold"))
        label.pack(side=tk.TOP, expand=True, padx=5, pady=5)

        def create_task_control_panel_frame() -> ttk.Frame:
            subframe = ttk.Frame(frame)

            def start_hand_guidance():
                self.kill_tasks()

                if self.meca_connected:
                    self.set_robot_motion_speed()
                    self.meca_robot.GripperOpen()
                    self.meca_robot.MovePose(170, -50, 180, 180, 0, -90)
                    self.meca_robot.WaitIdle()

                if self.add_task(HandGuidance):
                    self.run_task()
                    messagebox.showinfo("Info", "You can now move the robot by applying force on the sensor!")

            hand_guidance_button = ttk.Button(subframe, text="Hand Guidance", command=start_hand_guidance)
            hand_guidance_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)

            def start_button_testing():
                self.kill_tasks()
                if self.add_task(ButtonTesting):
                    self.run_task()
                    messagebox.showinfo("Info", "Button testing task started!")


            button_testing = ttk.Button(subframe, text="Button testing", command=start_button_testing)
            button_testing.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)

            def start_contour_following():
                self.kill_tasks()
                if self.add_task(ContourFollowing):
                    self.run_task()
                    messagebox.showinfo("Info", "Contour following task started!")

            contour_following_button = ttk.Button(subframe, text="Contour following", command=start_contour_following)
            contour_following_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)

            def pick_and_weigh():
                self.kill_tasks()
                if self.meca_connected:
                    self.set_robot_motion_speed()
                    self.meca_robot.GripperOpen()
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.MovePose(200, -88.5, 150, 0, 90, 0)
                    self.meca_robot.MovePose(218, -88.5, 70, 0, 90, 0)
                    self.meca_robot.WaitIdle()

                if not self.start_sensor():
                    return

                time.sleep(3.0)

                sample_count = 0
                force_empty_total = 0
                force_empty_average = 0
                while sample_count < 200:
                    start_time = time.perf_counter()
                    data_empty = self.sensor.read_frame()

                    sample_count += 1
                    force_empty_total += data_empty.force[0]
                    force_empty_average = force_empty_total / sample_count

                    time_diff = time.perf_counter() - start_time
                    time.sleep(max(1 / self.sensor_config_params["update_rate"] - time_diff, 0))

                # data_empty = self.sensor.read_frame()
                if self.meca_connected:
                    self.set_robot_motion_speed()
                    # self.meca_robot.MovePose(145, -65, 90, 90, 0, -90)
                    self.meca_robot.MoveLinRelTRF(11, 0, 0, 0, 0, 0)
                    self.meca_robot.GripperClose()
                    self.meca_robot.MoveLinRelTRF(-30, 0, 0, 0, 0, 0)
                    self.meca_robot.WaitIdle()

                time.sleep(3.0)

                sample_count = 0
                force_picked_total = 0
                force_picked_average = 0
                while sample_count < 200:
                    start_time = time.perf_counter()
                    data_picked = self.sensor.read_frame()

                    sample_count += 1
                    force_picked_total = force_picked_total + data_picked.force[0]
                    force_picked_average = force_picked_total / sample_count

                    time_diff = time.perf_counter() - start_time
                    time.sleep(max(1 / self.sensor_config_params["update_rate"] - time_diff, 0))

                # data_picked = self.sensor.read_frame()
                weight = (force_picked_average - force_empty_average)*1000/9.81
                # if self.add_task(pick_and_weigh):
                #     self.run_task()
                messagebox.showinfo("Info", "The weight is {:.2f} g".format(weight))

                if self.meca_connected:
                    self.set_robot_motion_speed()
                    self.meca_robot.MoveLinRelTRF(29.5, 0, 0, 0, 0, 0)
                    self.meca_robot.GripperOpen()
                    self.meca_robot.MoveLinRelTRF(-30, 0, 0, 0, 0, 0)
                    self.meca_robot.MovePose(200, -88.5, 150, 0, 90, 0)
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.WaitIdle()

                self.stop_sensor()

            pick_and_weigh_button = ttk.Button(subframe, text="Pick and Weigh", command=pick_and_weigh)
            pick_and_weigh_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)

            return subframe

        task_control_panel_frame = create_task_control_panel_frame()
        task_control_panel_frame.pack(side=tk.TOP, fill=tk.X, expand=True, padx=5, pady=5)

        def create_task_control_panel_frame_2() -> ttk.Frame:
            subframe = ttk.Frame(frame)

            def start_polishing_on_wheel():
                self.kill_tasks()
                if self.meca_connected:
                    self.set_robot_motion_speed()
                    self.meca_robot.MovePose(205, 50, 160, -135, 0, -90)
                    self.meca_robot.WaitIdle()

                if self.add_task(Polishing):
                    self.run_task()
                    messagebox.showinfo("Info", "Polishing nob task started!")

            polishing_button = ttk.Button(subframe, text="Polish nob", command=start_polishing_on_wheel)
            polishing_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)
            
            def start_deflashing():
                self.kill_tasks()
                if self.meca_connected:
                    self.set_robot_motion_speed()
                    self.meca_robot.GripperOpen()
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.MovePose(183, 65, 96, 180, 0, -90)
                    self.meca_robot.MovePose(183, 86, 96, 180, 0, -90)
                    self.meca_robot.MoveLinRelTRF(0, 0, 10.5, 0, 0, 0)
                    self.meca_robot.GripperClose()
                    self.meca_robot.MoveLinRelTRF(-30, 0, 0, 0, 0, 0)
                    self.meca_robot.MoveLinRelTRF(0, 0, -20, 0, 0, 0)
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.MovePose(190, 0, 150, 0, 90, 0)
                    self.meca_robot.MovePose(190, 0, 90, 0, 90, 0)
                    self.meca_robot.WaitIdle()

                if self.add_task(Deflashing):
                    self.run_task()
                    messagebox.showinfo("Info", "Deflashing task started!")

            deflashing_button = ttk.Button(subframe, text="Deflashing", command=start_deflashing)
            deflashing_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)

            def part_polishing():
                self.kill_tasks()
                if self.meca_connected:
                    self.set_robot_motion_speed()
                    self.meca_robot.GripperOpen
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.MovePose(140, 65, 96, 180, 0, -90)
                    self.meca_robot.MovePose(140, 86, 96, 180, 0, -90)
                    self.meca_robot.MoveLinRelTRF(0, 0, 10.5, 0, 0, 0)
                    self.meca_robot.GripperClose()
                    self.meca_robot.MoveLinRelTRF(-30, 0, 0, 0, 0, 0)
                    self.meca_robot.MoveLinRelTRF(0, 0, -20, 0, 0, 0)
                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.MovePose(200, 15, 150, 0, 90, 0)
                    self.meca_robot.MovePose(242, 15, 120, 0, 90, 0)
                    self.meca_robot.WaitIdle()

                if self.add_task(PartPolishing):
                    self.run_task()
                    messagebox.showinfo("Info", "Polishing housing task started!")

            part_polishing_button = ttk.Button(subframe, text="Polish housing", command=part_polishing)
            part_polishing_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)

            def start_alignment():
                self.kill_tasks()
                if self.meca_connected:
                    self.set_robot_motion_speed()
                    self.meca_robot.GripperOpen()

                    self.meca_robot.MovePose(170, 0, 180, 180, 0, -90)
                    self.meca_robot.MovePose(200, -88.5, 150, 0, 90, 0)
                    self.meca_robot.MovePose(218, -88.5, 70, 0, 90, 0)
                    self.meca_robot.MoveLinRelTRF(11, 0, 0, 0, 0, 0)
                    self.meca_robot.GripperClose()
                    self.meca_robot.MoveLinRelTRF(-30, 0, 0, 0, 0, 0)
                    self.meca_robot.MovePose(200, -88.5, 150, 0, 90, 0)
                    self.meca_robot.MovePose(196, -65, 75, 0, 90, 4)
                    self.meca_robot.WaitIdle()

                if self.add_task(Alignment):
                    self.run_task()
                    messagebox.showinfo("Info", "Alignment testing task started!")

            alignment_button = ttk.Button(subframe, text="Alignment", command=start_alignment)
            alignment_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)

            return subframe

        task_control_panel_frame_2 = create_task_control_panel_frame_2()
        task_control_panel_frame_2.pack(side=tk.TOP, fill=tk.X, expand=True, padx=5, pady=5)

        def stop_task():
            self.kill_tasks_go_home()
            messagebox.showinfo("Info", "Tasks stopped!")

        stop_task_button = ttk.Button(frame, text="Stop Task", command=stop_task)
        stop_task_button.pack(side=tk.TOP, fill=tk.X, expand=True, padx=5, pady=5)

        return frame

    def create_quit_button_frame(self, root: Union[ttk.Frame, tk.Tk]) -> ttk.Frame:
        frame = ttk.Frame(root)

        quit_button = ttk.Button(frame, text="Quit", command=self.stop_sequence)
        quit_button.pack(side=tk.TOP, fill=tk.X, expand=True, padx=5, pady=5)

        return frame

    def organize_widgets(self):

        plotter_frame = self.create_plotter_frame(self)
        plotter_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        def create_panel_frame() -> ttk.Frame:
            subframe = ttk.Frame(self)

            bota_logo_frame = self.create_bota_logo_frame(subframe)
            bota_logo_frame.pack(side=tk.TOP, fill=tk.X)

            status_frame = self.create_status_frame(subframe)
            status_frame['borderwidth'] = 5
            status_frame['relief'] = tk.GROOVE
            status_frame.pack(side=tk.TOP, fill=tk.X, expand=True, padx=5, pady=5)

            sensor_config_frame = self.create_sensor_config_frame(subframe)
            sensor_config_frame['borderwidth'] = 5
            sensor_config_frame['relief'] = tk.GROOVE
            sensor_config_frame.pack(side=tk.TOP, fill=tk.X, expand=True, padx=5, pady=5)

            meca_config_frame = self.create_meca_config_frame(subframe)
            meca_config_frame['borderwidth'] = 5
            meca_config_frame['relief'] = tk.GROOVE
            meca_config_frame.pack(side=tk.TOP, fill=tk.X, expand=True, padx=5, pady=5)

            task_selection_frame = self.create_task_selection_frame(subframe)
            task_selection_frame['borderwidth'] = 5
            task_selection_frame['relief'] = tk.GROOVE
            task_selection_frame.pack(side=tk.TOP, fill=tk.X, expand=True, padx=5, pady=5)

            quit_button_frame = self.create_quit_button_frame(subframe)
            quit_button_frame.pack(side=tk.BOTTOM, fill=tk.X, expand=True, padx=5, pady=5)

            return subframe

        panel_frame = create_panel_frame()
        panel_frame.pack(side=tk.LEFT, expand=False, fill=tk.BOTH)


if __name__ == "__main__":
    bota_window = BotaWindow()
    bota_window.mainloop()
