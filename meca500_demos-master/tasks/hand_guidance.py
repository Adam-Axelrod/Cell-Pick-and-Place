import sys
import os
package_path = os.path.dirname(os.path.dirname(__file__))
sys.path.append(package_path)

from typing import Optional, Dict
import numpy as np
from numpy import linalg as LA
import time
from multiprocessing import Event
from threading import Thread, Lock
import queue
import tkinter as tk
from tasks.task import Task
# try-import helper (robust to running as script or package)
try:
    from utils.helpers import limit
except Exception:
    # fallback: import by path if package not found
    helpers_path = os.path.join(project_root, "utils")
    if helpers_path not in sys.path:
        sys.path.insert(0, helpers_path)
    from helpers import limit

# ...existing code...

class HandGuidance(Task):
    def __init__(self, sensor, robot, sampling_period: float | None = None):
        super().__init__(sensor, robot, sampling_period)
        self.stop_event = Event()
        self.master_robot = robot
        self.bftSensor = sensor
        self.frequency = 800
        if getattr(self, "sampling_period", None) is not None:
            try:
                self.frequency = int(round(1.0 / float(self.sampling_period)))
            except Exception:
                pass
        self.robot_control_thread = None
        self.control_lock = Lock()
        self.command_queue = queue.Queue()

        # Hand guidance params
        self.gain_tr = 10
        self.gain_rot = 50
        self.alpha = 1
        self.f_threshold_high = 0.2
        self.f_threshold_low = 0.05
        self.m_threshold_high = 0.25
        self.m_threshold_low = 0.05
        self.wrench_filtered = np.zeros(6)
        self.wrench = np.zeros(6)

    def control_thread(self):
        while not self.stop_event.is_set():
            try:
                start_time = time.perf_counter()
                
                with self.control_lock:
                    data = self.bftSensor.read_frame()
                    if data is None:
                        continue
                    self.wrench[0:3] = data.force
                    self.wrench[3:] = data.torque

                    twist = self.hand_guidance_controller()
                    
                    # Put robot commands in queue instead of direct execution
                    self.command_queue.put(twist)

                time_diff = time.perf_counter() - start_time
                sleep_time = max(1 / self.frequency - time_diff, 0)
                if sleep_time > 0:
                    time.sleep(sleep_time)
            except Exception as e:
                print(f"Control thread error: {e}")
                break

    def process_robot_commands(self):
        """Process robot commands from queue - call this from main thread"""
        try:
            while not self.command_queue.empty():
                twist = self.command_queue.get_nowait()
                if twist is not None:
                    self.master_robot.MoveLinVelTRF(
                        twist[0], twist[1], twist[2], 
                        twist[3], twist[4], twist[5]
                    )
        except queue.Empty:
            pass
        except Exception as e:
            print(f"Error processing robot commands: {e}")

    def run(self):
        self.start_thread(self.control_thread)
        # Schedule regular processing of robot commands
        if hasattr(self, 'root') and isinstance(self.root, tk.Tk):
            self.root.after(10, self.process_robot_commands)

    def stop(self):
        self.default_stop()
        # Clear command queue
        while not self.command_queue.empty():
            try:
                self.command_queue.get_nowait()
            except queue.Empty:
                break

    def initialize(self) -> bool:
        if self.master_robot is not None:
            self.move_to_start()
            return True
        return False

    def control_thread(self):
        while not self.stop_event.is_set():
            start_time = time.perf_counter()
            data = self.bftSensor.read_frame()
            if data is None:
                continue
            self.wrench[0:3] = data.force
            self.wrench[3:] = data.torque

            twist = self.hand_guidance_controller()
            self.master_robot.MoveLinVelTRF(twist[0], twist[1], twist[2], twist[3], twist[4], twist[5])
            time_diff = time.perf_counter() - start_time
            time.sleep(max(1 / self.frequency - (time.perf_counter() - start_time), 0))

    def hand_guidance_controller(self):
        twist = np.zeros(6)
        motiongroup = "NONE"
        self.wrench_filtered = self.wrench_filtered + self.alpha * (self.wrench - self.wrench_filtered)

        normF = LA.norm(self.wrench[0:3])
        normM = LA.norm(self.wrench[3:])

        if normF <= self.f_threshold_low and normM < self.m_threshold_low:
            motiongroup = "NONE"

        if normM > self.m_threshold_high:
            motiongroup = "ROTATION"
        else:
            if motiongroup == "ROTATION" and normM > self.m_threshold_low:
                motiongroup = "ROTATION"
            elif normF > self.f_threshold_high:
                motiongroup = "TRANSLATION"
            elif motiongroup == "ROTATION" and normM < self.m_threshold_low:
                motiongroup = "NONE"
            elif motiongroup == "TRANSLATION" and normF < self.f_threshold_low:
                motiongroup = "NONE"

        match motiongroup:
            case ("NONE"):
                twist = np.zeros(6)
            case ("TRANSLATION"):
                tempDeadzone = (self.wrench_filtered[0:3] / normF * self.f_threshold_low)
                twist[0:3] = self.gain_tr * (self.wrench_filtered[0:3] - tempDeadzone)
                twist[3:] = np.zeros(3)
            case ("ROTATION"):
                tempDeadzone = (self.wrench_filtered[3:] / normF * self.m_threshold_low)
                twist[3:] = self.gain_rot * (self.wrench_filtered[3:] - tempDeadzone)
                twist[0:3] = np.zeros(3)

        for i in range(3):
            twist[i] = limit(twist[i], -60, 60)
            twist[i + 3] = limit(twist[i + 3], -45, 45)
        return twist
