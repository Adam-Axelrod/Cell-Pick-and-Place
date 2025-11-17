import sys
import os
import math

package_path = os.path.dirname(os.path.dirname(__file__))
sys.path.append(package_path)

import numpy as np
from numpy.typing import NDArray
from numpy import linalg as LA
import time
from multiprocessing import Event
from threading import Thread

from tasks.task import Task
from libraries.meca import robot_common, robot as rb
from simple_pid import PID
import tasks.functions
from tasks.functions import limit


class ContourFollowing(Task):
    def __init__(self, sensor, robot, sampling_period: float | None = None):
        super().__init__(sensor, robot, sampling_period)
        self.stop_event = Event()
        self.master_robot = robot
        self.bftSensor = sensor
        # default frequency; may be overridden by sampling_period injected by Task
        self.frequency = 800
        if getattr(self, "sampling_period", None) is not None:
            try:
                self.frequency = int(round(1.0 / float(self.sampling_period)))
            except Exception:
                pass
        self.robot_control_thread = None

        # Move to contact params
        self.force_threshold = 0.5
        self.force_setpoint = -3.0
        self.reference_frame = "WRF"
        self.approach_velocity = -5.0
        self.perpendicular_velocity = 5.0
        self.PID_params = [8.0, 0.01, 0.1]  # Kp, Ki, Kd
        self.max_distance = 50
        self.retract_distance = 0.

        self.start_pose = [170, -50, 180, 180, 0, -90]
        self.wrench = np.zeros(6)

    def move_to_start(self):
        self.setup_robot_defaults(move_pose=self.start_pose)

    def start_move_to_contact(self):
        self.run()
        try:
            while 1:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Stopped")
            self.stop()

    def run(self):
        self.start_thread(self.control_thread)

    def stop(self):
        self.default_stop()

    def initialize(self):
        if self.master_robot is not None:
            self.move_to_start()
            return True
        return False

    def control_thread(self):
        self.start_pose = np.array(self.master_robot.GetPose(timeout=3))
        self.start_pose = [0, 0, 0, 0, 0, 0]
        Fy_pid = PID(self.PID_params[0], self.PID_params[1], self.PID_params[2], setpoint=self.force_setpoint)
        direction = 1
        # print(self.max_distance)
        # print("start: ", self.start_pose[1])
        while not self.stop_event.is_set():

            start_time = time.perf_counter()
            data = self.bftSensor.read_frame()
            if data is None:
                continue
            self.wrench[0:3] = data.force
            self.wrench[3:] = data.torque
            ee_pose = np.array(self.master_robot.GetPose())

            contact_state = self.check_contact_state(self.wrench)
            # force control
            twist = np.zeros(6)

            # print(direction)
            # print(ee_pose[1])
            # print(self.start_pose[1] - self.max_distance)
            # print(self.start_pose[1] + self.max_distance)
            if ee_pose[1] < self.start_pose[1] - self.max_distance and direction == -1:
                direction = 1
            elif ee_pose[1] > self.start_pose[1] + self.max_distance and direction == 1:
                direction = -1

            # Handle different contact states
            if contact_state == "NO_CONTACT":
                # Move towards surface
                twist[2] = self.approach_velocity  # Approach in z direction
            elif contact_state == "CONTACT":
                # Force control
                twist[2] = Fy_pid(self.wrench[2])  # control in z direction
                twist[1] = direction * self.perpendicular_velocity  # Move along y in contact frame
            elif contact_state == "EXCESSIVE_FORCE":
                # Back off if force is too high
                twist[2] = 5.0  # Move away from surface

            # move robot
            self.master_robot.MoveLinVelWRF(twist[0], twist[1], twist[2], twist[3], twist[4],
                                            twist[5])

            time_diff = time.perf_counter() - start_time

            time.sleep(max(1 / self.frequency - time_diff, 0))

    def check_contact_state(self, wrench):
        """
        Check if the end effector is in contact with the surface
        Returns: "NO_CONTACT", "CONTACT", or "EXCESSIVE_FORCE"
        """
        # Get the norm of the force component (first 3 elements of wrench)
        force_magnitude = LA.norm(wrench[:3])

        if force_magnitude < self.force_threshold:
            return "NO_CONTACT"
        elif force_magnitude > self.force_threshold * 30.0:
            return "EXCESSIVE_FORCE"
        else:
            return "CONTACT"