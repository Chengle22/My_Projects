from vehicle import Car
from controller import Robot, Motor, DistanceSensor
from vehicle import Driver

csci3302_lab5_supervisor.init_supervisor()
robot = csci3302_lab5_supervisor.supervisor

SIM_TIMESTEP = int(robot.getBasicTimeStep())


while robot.step(SIM_TIMESTEP) != -1:
    print("YOO")