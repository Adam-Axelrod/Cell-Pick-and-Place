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


class Polishing(Task):
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
        self.force_threshold = 0.
        self.direction = np.zeros(3)
        self.reference_frame = "WRF"
        self.velocity = 0.
        self.max_distance = 0.
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
        self.start_pose = np.array(self.master_robot.GetPose())
        t_0 = time.perf_counter()
        Fy_pid = PID(2.0, 0.01, 0.01, setpoint=2)
        while not self.stop_event.is_set():

            start_time = time.perf_counter()
            data = self.bftSensor.read_frame()
            if data is None:
                continue
            self.wrench[0:3] = data.force
            self.wrench[3:] = data.torque

            ee_pose = np.array(self.master_robot.GetPose())
            rot_t_c = np.zeros(3)
            rot_t_c[0] = ee_pose[3] - self.rot_w_c[0]
            rot_t_c[1] = ee_pose[4] - self.rot_w_c[1]
            rot_t_c[2] = ee_pose[5] - self.rot_w_c[2]

            wrench_c = tasks.functions.wrench_transform(self.wrench, rot_t_c, self.sensor_offset)

            # force control
            twist_c = np.zeros(6)
            twist_c[1] = Fy_pid(wrench_c[1])  # control in y direction in the fixed contact frame

            # rotate twist to world frame
            twist_w = tasks.functions.twist_transform(twist_c, self.rot_w_c, [0, 0, 0])  # static transformation
            twist_w[5] = 18 * math.cos((start_time - t_0)*2.5)

            twist_trf = tasks.functions.twist_transform(twist_w, -ee_pose[3:], self.trf_offset)
            # move robot
            self.master_robot.MoveLinVelTRF(twist_trf[0], twist_trf[1], twist_trf[2], twist_trf[3], twist_trf[4],
                                            twist_trf[5])

            time_diff = time.perf_counter() - start_time

            time.sleep(max(1 / self.frequency - time_diff, 0))
            if (start_time - t_0) > 20:  # 20 seconds
                # restart polishing
                self.master_robot.MovePose(205, 50, 160, -135, 0, -90)
                self.master_robot.WaitIdle()
                t_0 = time.perf_counter()

