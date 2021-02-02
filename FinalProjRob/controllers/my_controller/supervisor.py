import copy
from controller import Supervisor
import numpy as np
import math


# create the Robot instance.

# get the time step of the current world.
supervisor = None
robot_node = None

def init_supervisor():
    global supervisor, robot_node, target_node

    # create the Supervisor instance.
    supervisor = Supervisor()

    # do this once only
    root = supervisor.getRoot() 
    root_children_field = root.getField("children") 
    robot_node = None
    for idx in range(root_children_field.getCount()):
        #print(root_children_field.getMFNode(idx).getFromProtoDef("LincolnMKZ"))
        if root_children_field.getMFNode(idx).getDef() == "LincolnMKZ":
            robot_node = root_children_field.getMFNode(idx)
    print(robot_node)
    return robot_node