"""thesupervisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor

# create the Robot instance.

# get the time step of the current world.
supervisor = None
robot_node = None
def init_supervisor():
    global supervisor, robot_node, target_node

    # create the Supervisor instance.
    supervisor = Supervisor()
    root = supervisor.getRoot()
    root_children_field = supervisor.getSelf() 
    # do this once only
    # root = supervisor.getRoot() 
    # root_children_field = root.getField("children") 
    # robot_node = None
    # target_node = None
    # for idx in range(root_children_field.getCount()):
        # if root_children_field.getMFNode(idx).getDef() == "TeslaModel3":
            # robot_node = root_children_field.getMFNode(idx)
        # if root_children_field.getMFNode(idx).getDef() == "Goal":
            # target_node = root_children_field.getMFNode(idx) 

    # start_translation = copy.copy(robot_node.getField("translation").getSFVec3f())
    # start_rotation = copy.copy(robot_node.getField("rotation").getSFRotation())


# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller

