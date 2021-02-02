from controller import Robot, DistanceSensor, GPS
from vehicle import Car
import math
import os
import csci3302_lab5_supervisor
from os import path
from vehicle import Driver
import numpy as np
import random
import operator

class Node:
    """
    Node for RRT/RRT* Algorithm. This is what you'll make your graph with!
    """
    def __init__(self, pt, disttogoal, direction, vel, t, parent=None):
        self.point = pt # n-Dimensional point
        self.parent = parent # Parent node
        self.path_from_parent = [] # List of points along the way from the parent node (for visualization)
        self.vel = vel
        self.t = t
        self.disttogoal = disttogoal
        self.direction = direction

class AgentNode:
    def __init__(self, pt, direction, goal,  parent=None):
        self.point = pt # n-Dimensional point
        self.parent = parent # Parent node
        self.path_from_parent = [] # List of points along the way from the parent node (for visualization)
        self.direction = direction
        self.goal = goal
        self.node_list = []
        self.timelist = []
        self.agentname = ""
        if(self.direction == "left"):
                self.coordstogoal = abs(self.point[1] - self.goal)
        elif(self.direction == "right"):
                self.coordstogoal = abs(self.point[1] - self.goal)
        elif(self.direction == "top"):
                self.coordstogoal = abs(self.point[0] - self.goal ) 
        elif(self.direction == "bot"):
                self.coordstogoal = abs(self.point[0] - self.goal ) 
        self.path = []
        
class RRTsolver:
    def __init__(self):
        self.agents = []
        self.closestagent = AgentNode((0,0), "right", 0)
        self.furthestagent = AgentNode((0,0), "right", 0)
        self.accspeed = .9 * 5723.11379618 #IN C p S p S
        self.maxspeed = 170 #in KPH
        self.timetogoal = 0
        self.timeintervals = ()
        self.obstacles = {}
        self.speedlist = [1*170*0.2500002007, .9*170*0.2500002007, .8*170*0.2500002007, .7*170*0.2500002007, .6*170*0.2500002007, .5*170*0.2500002007, .4*170*0.2500002007, .3*170*0.2500002007, .2*170*0.2500002007, .1*170*0.2500002007]
        self.randchoicebias = [10/55,9/55, 8/55,7/55,6/55,5/55,4/55,3/55,2/55,1/55]
        self.timeinterval = 0
        #np.random.choice([1,2], p=[0.9, 0.1])
        self.reroutecounter = 0
        self.destruct = False
    def makeagentlist(self):
        f=open("agentsandinfo.txt", "r")
        if f.mode == 'r':
            contents =f.read()
        f.close()
        for i in contents.splitlines():
            j = i.split()
            agentlist = []
            for k in range(4):
                agentlist.append(j[k])
            if(agentlist[3] == "left"):
                goal = -9
            elif(agentlist[3] == "right"):
                goal = 9
            elif(agentlist[3] == "top"):
                goal = -8
            elif(agentlist[3] == "bot"):
                goal = 8
            newAgentNode = AgentNode((float(agentlist[1]),float(agentlist[2])), agentlist[3] , goal, None)
            newAgentNode.name = agentlist[0]
            self.agents.append(newAgentNode)
        return
        
    def findfurthestandclosestagent(self):
        currdist = 0
        closestdist = 99999
        furthestdist = 0
        for i in self.agents:
            if (i.direction == "left"):
                currdist = abs(i.point[1])
                if(currdist < closestdist):
                    closestdist = currdist
                    closestagent = i
                if(currdist > furthestdist):
                    furthestdist = currdist
                    furthestagent = i
            elif (i.direction == "right"):
                currdist = abs(i.point[1])
                if(currdist < closestdist):
                    closestdist = currdist
                    closestagent = i
                if(currdist > furthestdist):
                    furthestdist = currdist
                    furthestagent = i
            elif (i.direction == "top"):
                currdist = abs(i.point[0])
                if(currdist < closestdist):
                    closestdist = currdist
                    closestagent = i
                if(currdist > furthestdist):
                    furthestdist = currdist
                    furthestagent = i                              
            elif (i.direction == "bot"):
                currdist = abs(i.point[0])
                if(currdist < closestdist):
                    closestdist = currdist
                    closestagent = i
                if(currdist > furthestdist):
                    furthestdist = currdist
                    furthestagent = i
        self.closestagent = closestagent
        self.furthestagent = furthestagent               
            
    def RRTprepper(self):
        self.makeagentlist()
        self.findfurthestandclosestagent()
        self.timetogoal = timetoendworstcase(self.furthestagent.coordstogoal, self.accspeed)
        self.timeintervals = np.linspace(0, self.timetogoal, 100)
        self.timeinterval = self.timeintervals[1] - self.timeintervals[0]
    
    def get_random_valid_vertex(self, timeintervals, agent):
        randtime = random.choice(timeintervals)
        randspeed = np.random.choice(self.speedlist, p =self.randchoicebias)
        return((randtime, randspeed))
        
    def get_some_choices(self, agent, q_point):
        node_list = agent.node_list
        potentialsont = []
        actualpotentials = []
        for i in node_list:
            if(q_point[0] > i.t):
                potentialsont.append(i)
            if(q_point[0] <= i.t):
                break
        #print(i.t)
        for i in potentialsont:
            if(len(actualpotentials) >=1):
                break
            coordscovered = distfromacc(i.vel, self.accspeed, q_point[1], q_point[0] - i.t)    
            disttogoal = i.disttogoal-coordscovered    
            #Find the new point:
            if(i.direction == "left"):
                newpoint = (i.point[0],i.point[1]+coordscovered)
            if(i.direction == "right"):
                newpoint = (i.point[0],i.point[1]-coordscovered)
            if(i.direction == "top"):
                newpoint = (i.point[0]-coordscovered,i.point[1])
            if(i.direction == "bot"):
                newpoint = (i.point[0]+coordscovered,i.point[1])
            newnode = Node(newpoint, disttogoal, i.direction, q_point[1], q_point[0])
            newnode.vel = distfromaccreturnvel(i.vel, self.accspeed, q_point[1], q_point[0] - i.t)
            newnode.parent = i
            #pathtoparenttuples = ((x, y), t)
            pathtoparent = []
            if(newnode not in agent.node_list):
                for j in agent.timelist:
                    if (j >= newnode.parent.t and j<= newnode.t):
                        coordscovered = distfromacc(i.vel, self.accspeed, q_point[1], j-newnode.parent.t)
                        if(i.direction == "left"):
                                newpoint = (i.point[0],i.point[1]+coordscovered)
                        if(i.direction == "right"):
                                newpoint = (i.point[0],i.point[1]-coordscovered)
                        if(i.direction == "top"):
                                newpoint = (i.point[0]-coordscovered,i.point[1])
                        if(i.direction == "bot"):
                                newpoint = (i.point[0]+coordscovered,i.point[1]) 
                        pathtoparent.append((newpoint, j))
            newnode.pathtoparent = pathtoparent
            actualpotentials.append(newnode)
        return(actualpotentials)
        
        
    def RRTsolvetime(self):
        self.agents.sort(key = lambda x:x.coordstogoal)
        for i in self.agents:
            self.singlepathsolve(i)
        print("number of reroutes probs doesnt work",self.reroutecounter)
        for i in self.agents:
            newfilename = i.name +".txt"
            f= open(newfilename,"a")
            for j in i.path:
                calcvel = j.vel/0.2500002007
                f.write(str(calcvel))
                f.write(' ')
                f.write(str(j.t))
                f.write("\n")
            f.close()
        return
        
    def isviable(self, potentialnode):
        for i in potentialnode.pathtoparent:
            if(i[1] not in self.obstacles.keys()):
                pass
            else:
                #print("here")
                for j in self.obstacles[i[1]]:
                    #print(self.obstacles[i[1]])
                    # print(j)
                    print(potentialnode.direction)
                    margin = 6
                    maxx = j[0] + margin
                    maxy = j[1] + margin
                    minx = j[0] - margin
                    miny = j[1] - margin
                    x = round(i[0][0])
                    y = round(i[0][1])
                    if(x <= maxx and x >= minx and y <= maxy and y>=miny):
                        self.reroutecounter = self.reroutecounter + 1
                        return False
        return True
    
    def singlepathsolve(self, agent):
        #__init__(self, pt, disttogoal, vel, t, parent=None):
        agent.node_list.append(Node(pt = agent.point, disttogoal = agent.coordstogoal, direction = agent.direction, vel = 0, t = 0))
        done = False
        worstcase = timetoendworstcase(agent.coordstogoal, self.accspeed)
        for i in range(len(self.timeintervals)):

            if(i == 0):
                pass
            elif(self.timeintervals[i] <= worstcase):
                agent.timelist.append(self.timeintervals[i])
            if(i == len(self.timeintervals)-1):
                break
            if(self.timeintervals[i] <= worstcase and self.timeintervals[i+1] >= worstcase):
                agent.timelist.append(self.timeintervals[i+1])
                break
        counter = 0
        while done == False:
              agent.node_list.sort(key = lambda x:x.t)
              agent.node_list.reverse()
              qrand = self.get_random_valid_vertex(agent.timelist, agent)
              listofnewviablenodes = self.get_some_choices(agent, qrand)
              for i in listofnewviablenodes:
                  if(self.isviable(i) == True):
                      agent.node_list.append(i)
              # for i in agent.node_list:
                  # print(i.point, i.disttogoal, i.direction, i.vel, i.t)
              #print(agent.direction, counter)
              if(counter == 100000):
                  print("no solution")
                  return
              counter = counter+1
              if(agent.node_list[-1].disttogoal <=0):
                  route = []
                  currnode = agent.node_list[-1]
                  while(currnode.parent is not None):
                      route.append(currnode)
                      currnode = currnode.parent
                  for i in route:
                      for j in i.pathtoparent:
                          if(j[1] not in self.obstacles.keys()):
                              self.obstacles[j[1]] = [(round(j[0][0]), round(j[0][1]))]
                          else:
                              self.obstacles[j[1]].append((round(j[0][0]), round(j[0][1])))
                  agent.path = route
                  return
        
        
def timetoendworstcase(coordstogoal, ACCSPEED):
    velInitial = 0
    velFinal = 10
    UNITIZEDvelInit = KPHtoCOORDSPS(velInitial)
    UNITIZEDvelFinal = KPHtoCOORDSPS(velFinal)
    timetoreachfinal = UNITIZEDvelFinal/ACCSPEED
    unitsmovedduraccel = UNITIZEDvelInit * timetoreachfinal + timetoreachfinal * 1/2 *(UNITIZEDvelFinal - UNITIZEDvelInit)
    timetogoal = (coordstogoal - unitsmovedduraccel)/UNITIZEDvelFinal
    return timetogoal
    
    
    
    
    
    
    
    
    
    
    
    
def KPHtoCOORDSPS(number):
    return(number * 1000 *(1/3600) * .9)


def distfromacc(velInitial, ACCSPEED, velFinal, time): #Time in seconds
    UNITIZEDvelInit = KPHtoCOORDSPS(velInitial)
    UNITIZEDvelFinal = KPHtoCOORDSPS(velFinal)
    if(velInitial>velFinal):
        accelerate = False
    else:
        accelerate = True
    if(accelerate == True):
        trueSpeedFinal = UNITIZEDvelInit + ACCSPEED * time
    else:
        trueSpeedFinal = UNITIZEDvelInit - ACCSPEED * time
    timetoreachfinal = UNITIZEDvelFinal/ACCSPEED
    timeatfinal = time - timetoreachfinal
    if(trueSpeedFinal >= UNITIZEDvelFinal or trueSpeedFinal <= UNITIZEDvelFinal):
        unitsmoved = timeatfinal * UNITIZEDvelFinal + UNITIZEDvelInit * timetoreachfinal + timetoreachfinal * 1/2 *(UNITIZEDvelFinal - UNITIZEDvelInit)
    else:
        unitsmoved = UNITIZEDvelInit * time + time * 1/2 *(trueSpeedFinal - UNITIZEDvelInit)
    return(unitsmoved)
    
def distfromaccreturnvel(velInitial, ACCSPEED, velFinal, time):
    UNITIZEDvelInit = KPHtoCOORDSPS(velInitial)
    UNITIZEDvelFinal = KPHtoCOORDSPS(velFinal)
    if(velInitial>velFinal):
        accelerate = False
    else:
        accelerate = True
    if(accelerate == True):
        trueSpeedFinal = UNITIZEDvelInit + ACCSPEED * time
    else:
        trueSpeedFinal = UNITIZEDvelInit - ACCSPEED * time
    timetoreachfinal = UNITIZEDvelFinal/ACCSPEED
    timeatfinal = time - timetoreachfinal
    if(trueSpeedFinal >= UNITIZEDvelFinal or trueSpeedFinal <= UNITIZEDvelFinal):
        unitsmoved = timeatfinal * UNITIZEDvelFinal + UNITIZEDvelInit * timetoreachfinal + timetoreachfinal * 1/2 *(UNITIZEDvelFinal - UNITIZEDvelInit)
        unitsmoved = UNITIZEDvelFinal
    else:
        unitsmoved = UNITIZEDvelInit * time + time * 1/2 *(trueSpeedFinal - UNITIZEDvelInit)
        unitsmoved = trueSpeedFinal
    return(unitsmoved)   
    
    
    
    
    
    
def RRT(MAXSPEED, ACCSPEED):
    # accspeed = ACCTOKP(ACCSPEED)
    # time = 1
    # distfromacc(0, accspeed, MAXSPEED, time)
    
    f=open("agentsandinfo.txt", "r")
    if f.mode == 'r':
        contents =f.read()
    f.close()
    for i in contents.splitlines():
        j = i.split()
        agentlist = []
        for k in range(4):
            agentlist.append(j[k])
        newAgentNode = AgentNode((float(agentlist[1]),float(agentlist[2])), agentlist[3] , None)
    return
