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


class Alignment(Task):
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
        self.targe_force = -1.5

        self.force_threshold = 0.5
        self.reference_frame = "WRF"
        self.velocity = 4.0
        self.max_distance = 50
        self.retract_distance = 0.

        self.start_pose = np.zeros(6)
        self.surface_pose = np.zeros(6)
        self.reached_surface = False

        self.alpha = 1
        self.wrenchAverage = np.zeros(6)
        self.wrenchTotal = np.zeros(6)
        self.wrench = np.zeros(6)
        self.pid = PID(1, 0.1, 0.05, setpoint=0)

        self.sensor_offset = np.array([0, 0, 0.03])  # m offset from sensor to TCP
        self.trf_offset = np.array([0, 0, -55])  # mm offset to contact point
        self.rot_w_c = np.array([-45, 0, 0])  # rotation from contact frame to world frame

    def move_to_start(self):
        print("Move to home")
        self.setup_robot_defaults()
        # self.master_robot.MovePose(170, 0, 180, 180, 0, 0)

    def start_move_to_contact(self):
        self.run()
        try:
            while 1:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Stopped")
            self.stop()

    def run(self):
        self.robot_control_thread = Thread(target=self.control_thread)
        self.robot_control_thread.start()

    def stop(self):
        self.stop_event.set()
        if self.robot_control_thread:
            self.robot_control_thread.join()
        self.bftSensor = None

    def initialize(self):
        if self.master_robot is not None:
            self.move_to_start()
            return True
        return False

    def control_thread(self):
        print("Find corner")
        self.reached_surface = False
        curr_pose = np.array(self.master_robot.GetPose())
        target_pose = np.array(self.master_robot.GetPose())
        for i in range(1):
            print(i)

            direction = np.zeros(3)
            if i == 0:
                direction[2] = -1.0
            elif i == 1:
                direction[0] = -1.0
            else:
                direction[1] = 1.0

            while not self.stop_event.is_set():
                start_time = time.perf_counter()
                data = self.bftSensor.read_frame()
                if data is None:
                    continue
                self.wrench[0:3] = data.force
                self.wrench[3:] = data.torque

                twist = self.move_to_contact_controller(direction)
                curr_pose = self.master_robot.GetPose()

                if self.reached_surface:
                    break
                # if self.next:
                #     self.move_to_next()
                #     self.next = False
                #     self.reached_surface = False
                #     continue

                if self.reference_frame == "WRF":
                    self.master_robot.MoveLinVelWRF(twist[0], twist[1], twist[2], 0., 0., 0.)
                elif self.reference_frame == "TRF":
                    self.master_robot.MoveLinVelTRF(twist[0], twist[1], twist[2], 0., 0., 0.)
                time_diff = time.perf_counter() - start_time
                time.sleep(max(1 / self.frequency - time_diff, 0))

            print("Retract")
            target_pose = curr_pose
            target_pose[0] = target_pose[0] - 2.0*direction[0]
            target_pose[1] = target_pose[1] - 2.0*direction[1]
            target_pose[2] = target_pose[2] - 2.0*direction[2]
            self.master_robot.MovePose(target_pose[0], target_pose[1], target_pose[2], target_pose[3], target_pose[4], target_pose[5])
            self.master_robot.WaitIdle()
            self.reached_surface = False

        fx_pid = PID(1.5, 0.01, 0.01, setpoint=self.targe_force)
        print("Start alignment")
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

            # Handle different contact states
            if contact_state == "NO_CONTACT":
                # print("no contact")
                # Move towards surface
                twist[0] = self.velocity  # Approach in x direction
            elif contact_state == "CONTACT":
                # print("contact")
                # Force control
                twist[0] = -fx_pid(self.wrench[0])  # control in tool x direction
                twist[5] = (self.wrench[5])*150.0
                twist[1] = -twist[5]*0.0174532925*50.0
            elif contact_state == "EXCESSIVE_FORCE":
                # print("excessive force")
                # Back off if force is too high
                twist[0] = -2.0  # Move away from surface

            # move robot //TODO: generate motion in TRF
            self.master_robot.MoveLinVelTRF(twist[0], twist[1], twist[2], twist[3], twist[4],
                                            twist[5])

            time_diff = time.perf_counter() - start_time

            time.sleep(max(1 / self.frequency - time_diff, 0))

    def move_to_contact_controller(self, direction):
        twist = np.zeros(3)

        normF = LA.norm(self.wrench[0:3])
        # print(normF)

        # curr_pose = np.array(self.master_robot.GetPose())
        #
        # distance = LA.norm(curr_pose[0:3] - self.start_pose[0:3])

        # if distance > self.max_distance:
        #     print("Max distance reached")
        #     return twist

        if not self.reached_surface:
            twist = direction * self.velocity
            if normF > self.force_threshold:
                self.reached_surface = True
                self.surface_pose = np.array(self.master_robot.GetPose())
                print("Surface reached")

        # if self.reached_surface:
        #     twist = -self.direction * self.velocity
        #     if LA.norm(self.surface_pose[0:3] - curr_pose[0:3]) > self.retract_distance:
        #         twist = np.zeros(3)
        #         self.next = True
        #         print("Retracted")

        return twist

    def check_contact_state(self, wrench):
        """
        Check if the end effector is in contact with the surface
        Returns: "NO_CONTACT", "CONTACT", or "EXCESSIVE_FORCE"
        """
        # Get the norm of the force component (first 3 elements of wrench)
        force_magnitude = LA.norm(wrench[:3])

        if force_magnitude < self.force_threshold * 0.5:
            return "NO_CONTACT"
        elif force_magnitude > self.force_threshold * 15.0:
            return "EXCESSIVE_FORCE"
        else:
            return "CONTACT"
