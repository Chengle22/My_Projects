import math

from vehicle import Driver

driver = Driver()
driver.setSteeringAngle(0.2)
driver.setCruisingSpeed(20)
SIM_TIMESTEP = int(robot.getBasicTimeStep())
while driver.step() != -1:
  angle = 0.3 * math.cos(driver.getTime())
  driver.setSteeringAngle(angle)
  
 SIM_TIMESTEP = int(car.getBasicTimeStep())