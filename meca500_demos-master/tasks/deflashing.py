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


class Deflashing(Task):
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
        self.targe_force = -3.0

        self.force_threshold = 1.0
        self.direction = np.zeros(3)
        self.reference_frame = "WRF"
        self.velocity = 0.
        self.max_distance = 50
        self.retract_distance = 0.

        self.start_pose = np.zeros(6)
        self.surface_pose = np.zeros(6)
        self.reached_surface = False
        self.alpha = 1
        self.wrench_filtered = np.zeros(6)
        self.wrench = np.zeros(6)
        self.pid = PID(1, 0.1, 0.05, setpoint=0)

        self.sensor_offset = np.array([0, 0, 0.03])  # m offset from sensor to TCP
        self.trf_offset = np.array([0, 0, -55])  # mm offset to contact point
        self.rot_w_c = np.array([-45, 0, 0])  # rotation from contact frame to world frame

    def move_to_start(self):
        # maintain the debug print but use shared defaults
        print("Move to home")
        self.setup_robot_defaults()

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
        fy_pid = PID(3.0, 0.01, 0.1, setpoint=self.targe_force)
        direction = 1
        # print(self.max_distance)
        # print("start: ", self.start_pose[1])
        print("Start control thread")
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
                # print("no contact")
                # Move towards surface
                twist[2] = -5.0  # Approach in z direction
            elif contact_state == "CONTACT":
                # print("contact")
                # Force control
                twist[2] = fy_pid(self.wrench[0])  # control in tool x direction
                twist[1] = direction * 5.0  # Move along y in contact frame
            elif contact_state == "EXCESSIVE_FORCE":
                # print("excessive force")
                # Back off if force is too high
                twist[2] = 2.0  # Move away from surface

            # move robot //TODO: generate motion in TRF
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

        if force_magnitude < self.force_threshold * 0.5:
            return "NO_CONTACT"
        elif force_magnitude > self.force_threshold * 30.0:
            return "EXCESSIVE_FORCE"
        else:
            return "CONTACT"
