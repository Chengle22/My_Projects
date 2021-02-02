from controller import Robot, DistanceSensor
from vehicle import Car
import math

from vehicle import Driver

TIME_STEP = 32

robot = Robot()
TIME_STEP = 32
name = robot.getName()
n_devices = robot.getNumberOfDevices()
print("number of devices", n_devices)
while robot.step(TIME_STEP) != -1:
    pass