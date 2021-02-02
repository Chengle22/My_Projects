from controller import Robot, DistanceSensor, GPS, Compass, Supervisor, Node
from vehicle import Car
import math
import os
from os import path
from vehicle import Driver
from magic import *
import supervisor
import time
import numpy as np


TIME_STEP = 32
MAXSPEED = 170
ACCSPEED = 0.6438503020702058
ACCSPEEDinCOORDSSS = .9 * 5723.11379618
#nodechild = supervisor.init_supervisor()

#driver = supervisor.supervisor
#driver = Supervisor()
#driver = supervisor.supervisor
driver = Car()
sensor = driver.getGPS("gps")
sensor.enable(1)
name = driver.getName()
#print(nodechild.getVelocity())
driver.setSteeringAngle(0)
#nodechild.setVelocity([10,10,10,10,10,10,10])
driver.setCruisingSpeed(0)
print(name)
if os.path.exists("agents.txt") and name == "first":
    os.remove("agents.txt")
f= open("agents.txt","a")
f.write(name)
f.write("\n")
f.close()
if(name == "last"):
   with open("agents.txt") as f:
       count = sum(1 for _ in f)
   if os.path.exists("numberofagents.txt"):
        os.remove("numberofagents.txt")
   f = open("numberofagents.txt","a")
   f.write(str(count))
   f.close()
f=open("numberofagents.txt", "r")
if f.mode == 'r':
    contents =f.read()
    count = int(contents)
f.close()
driver.step()
directioncheck = sensor.getValues()
if(round(directioncheck[0]*100000,2) <= -6.0):
    direction = "bot"
elif(round(directioncheck[0]*100000,2) >=6.6):
    direction = "top"
elif round(directioncheck[1]*100000,2) >=0:
    direction = "right"
else:
    direction = "left"
if os.path.exists("agentsandinfo.txt") and name == "first":
    os.remove("agentsandinfo.txt")
f= open("agentsandinfo.txt","a")
f.write(name)
f.write(' ')
f.write(str(round(directioncheck[0]*100000,2)))
f.write(' ')
f.write(str(round(directioncheck[1]*100000,2)))
f.write(' ')
f.write(direction)
f.write("\n")
f.close()

if(name == "last"):
    time.sleep(1)
    solution = RRTsolver()
    solution.RRTprepper()
    solution.RRTsolvetime()
# MAKE SURE TO ADD A SLEEP FUNCTION SO IF THE FINAL FILE
#DOES NOT EXIST HERE IT DOESN'T CONTINUE TILL THEN MANUALLY DELETE
#AGENTS AND INFO
# if os.path.exists("agentsandinfo.txt"):
    # os.remove("agentsandinfo.txt")
#print("expected change", distfromacc(0, ACCSPEEDinCOORDSSS, 10, 3.2))
prevveloc = driver.getCurrentSpeed()
if(name == "first"):
     currval = sensor.getValues()
     #print("initial vals", round(currval[1]*100000,2))
myfile = name+ ".txt"
timeplan = []
time.sleep(2)
f=open(myfile, "r")
if f.mode == 'r':
    contents =f.read()
f.close()
for i in contents.splitlines():
    j = i.split()
    timeplan.append((float(j[0]), float(j[1])))
if(os.path.exists("agentsandinfo.txt") and name == "last"):
    os.remove("agentsandinfo.txt")
time.sleep(1)
#os.remove(myfile)
times = []
speeds = []
for i in timeplan:
    times.append(i[1])
    speeds.append(i[0])
index = 0
if(len(speeds) == 0):
    driver.setCruisingSpeed(170)
else:
    driver.setCruisingSpeed(int(speeds[0]))
while driver.step() != -1:
    if(len(speeds) == 0):
        pass
    elif(times[index] > driver.getTime() and index < len(times)-1):
        index = index + 1
        driver.setCruisingSpeed(int(speeds[index]))
    currval = sensor.getValues()
    prevveloc = driver.getCurrentSpeed()
    #print(driver.getCurrentSpeed())
    if(name == "first"):
        pass
        #print( round(currval[0]*100000,2))
        # print(driver.getTime(), round(currval[1]*100000,2))
       
        #print(driver.getTime(), driver.getCurrentSpeed(),round(currval[0]*100000,2), round(currval[1]*100000,2))
        #print(driver.getTime(), driver.getCurrentSpeed(), round(currval[1]*100000,2))