import sys
import os

package_path = os.path.dirname(os.path.dirname(__file__))
sys.path.append(package_path)

from typing import Optional, Dict
import numpy as np
from numpy.typing import NDArray
from numpy import linalg as LA
import time
from multiprocessing import Event
from threading import Thread

from tasks.task import Task
from libraries.meca import robot_common, robot as rb
from tasks.functions import limit


class ButtonTesting(Task):

    def __init__(self, sensor, robot, sampling_period: float | None = None):
        super().__init__(sensor, robot, sampling_period)
        self.stop_event = Event()
        self.master_robot = robot
        self.bftSensor = sensor
        # default frequency; may be overridden by sampling_period injected by Taskb
        self.frequency = 800
        if getattr(self, "sampling_period", None) is not None:
            try:
                self.frequency = int(round(1.0 / float(self.sampling_period)))
            except Exception:
                pass
        self.robot_control_thread = None

        # Move to contact params
        self.force_threshold = 1.0
        self.direction = np.array([0, 0, -1])
        self.reference_frame = "WRF"
        self.velocity = 3
        self.max_distance = -10
        self.retract_distance = 8

        self.start_pose = [182, -68, 105, 180, 0, -90]

        self.surface_pose = np.zeros(6)
        self.reached_surface = False
        self.next = False
        self.x_counter = 0
        self.y_counter = 1

        self.alpha = 1
        self.wrench_filtered = np.zeros(6)
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
        # self.start_pose = np.array(self.master_robot.GetPose())
        while not self.stop_event.is_set():
            start_time = time.perf_counter()
            data = self.bftSensor.read_frame()
            if data is None:
                continue
            self.wrench[0:3] = data.force
            self.wrench[3:] = data.torque

            twist = self.move_to_contact_controller()
            if self.next:
                self.move_to_next()
                self.next = False
                self.reached_surface = False
                continue

            if self.reference_frame == "WRF":
                self.master_robot.MoveLinVelWRF(twist[0], twist[1], twist[2], 0., 0., 0.)
            elif self.reference_frame == "TRF":
                self.master_robot.MoveLinVelTRF(twist[0], twist[1], twist[2], 0., 0., 0.)
            time_diff = time.perf_counter() - start_time
            time.sleep(max(1 / self.frequency - time_diff, 0))

    def move_to_contact_controller(self):
        twist = np.zeros(3)

        normF = LA.norm(self.wrench[0:3])

        curr_pose = np.array(self.master_robot.GetPose())

        distance = LA.norm(curr_pose[0:3] - self.start_pose[0:3])

        # if distance > self.max_distance:
        #     print("Max distance reached")
        #     return twist

        if not self.reached_surface:
            twist = self.direction * self.velocity
            if normF > self.force_threshold:
                self.reached_surface = True
                self.surface_pose = np.array(self.master_robot.GetPose())
                print("Surface reached")

        if self.reached_surface:
            twist = -self.direction * self.velocity
            if LA.norm(self.surface_pose[0:3] - curr_pose[0:3]) > self.retract_distance:
                twist = np.zeros(3)
                self.next = True
                print("Retracted")

        return twist

    def move_to_next(self):
        pose = np.copy(self.start_pose)
        # print("start pose; ", self.start_pose)
        pose[0] = self.start_pose[0] + 18 * self.x_counter
        pose[1] = self.start_pose[1] + 18 * self.y_counter
        # print(self.x_counter)
        # print(self.y_counter)
        # print(pose)
        self.master_robot.MovePose(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])

        self.y_counter += 1
        if self.y_counter > 2:
            self.y_counter = 0
            self.x_counter += 1

        if self.x_counter > 2:
            self.x_counter = 0
            self.y_counter = 0
