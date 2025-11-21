# Run this in Python to check
from lerobot.robots.robot import Robot
from lerobot.teleoperators.teleoperator import Teleoperator

print("Available robots:", Robot.__subclasses__)
print("Available teleoperators:", Teleoperator.__subclasses__())