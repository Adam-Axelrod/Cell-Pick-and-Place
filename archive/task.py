import sys
import os

package_path = os.path.dirname(os.path.dirname(__file__))
sys.path.append(package_path)

from abc import ABC, abstractmethod

class Task(ABC):
    @abstractmethod
    def __init__(self, sensor, robot, sampling_period: float | None = None):
        """Base Task constructor.

        sampling_period: optional control loop period in seconds (1 / sampling_rate).
        Subclasses should call super().__init__(...) if they override __init__.
        """
        # store sampling period (seconds); if None, task may use its default
        self.sampling_period = sampling_period

    @abstractmethod
    def run(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def initialize(self) -> bool:
        pass

    # Convenience helpers to reduce duplicate boilerplate in Task subclasses
    def setup_robot_defaults(self, move_pose: list | None = None):
        """Apply default robot motion settings used across demos.

        If `move_pose` is provided it will call `MovePose` with the pose values.
        """
        robot = getattr(self, "master_robot", None)
        if robot is None:
            return
        robot.ClearMotion()
        robot.ResetError()
        robot.ResumeMotion()
        # defaults mirrored from existing tasks
        try:
            robot.SetVelTimeout(0.01)
            robot.SetJointAcc(30)
            robot.SetJointVel(25)
            robot.SetJointAcc(150)
            robot.SetCartAcc(600)
        except Exception:
            # Some robot wrappers may not expose all methods; ignore failures here
            pass
        if move_pose:
            # expect an iterable with 6 elements
            try:
                robot.MovePose(*move_pose)
            except Exception:
                pass

    def start_thread(self, target):
        """Start a control thread and store it on self.robot_control_thread."""
        self.robot_control_thread = __import__("threading").Thread(target=target)
        self.robot_control_thread.start()

    def default_stop(self):
        """Common stop procedure: set stop_event, join thread and clear sensor reference."""
        stop_event = getattr(self, "stop_event", None)
        if stop_event is not None:
            try:
                stop_event.set()
            except Exception:
                pass
        t = getattr(self, "robot_control_thread", None)
        if t is not None:
            try:
                t.join()
            except Exception:
                pass
        if getattr(self, "bftSensor", None) is not None:
            try:
                self.bftSensor = None
            except Exception:
                pass
