"""supervisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import copy
from controller import Supervisor
import numpy as np
import math


supervisor = None
robot_node = None
target_node = None

def init_supervisor():
    global supervisor, robot_node, target_node
    global start_translation, start_rotation
    # create the Supervisor instance.
    supervisor = Supervisor()

    # do this once only
    root = supervisor.getRoot() 
    root_children_field = root.getField("children") 
    robot_node = None
    target_node = None
    for idx in range(root_children_field.getCount()):
        if root_children_field.getMFNode(idx).getDef() == "Player":
            robot_node = root_children_field.getMFNode(idx)
        if root_children_field.getMFNode(idx).getDef() == "Goal":
            target_node = root_children_field.getMFNode(idx) 

    start_translation = copy.copy(robot_node.getField("translation").getSFVec3f())
    start_rotation = copy.copy(robot_node.getField("rotation").getSFRotation())


def supervisor_reset_to_home():
    global robot_node
    global start_rotation, start_translation
    pos_field = robot_node.getField("translation")
    pos_field.setSFVec3f(start_translation)
    pos_field = robot_node.getField("rotation")
    pos_field.setSFRotation(start_rotation)
    supervisor.resetPhysics()
    print("Supervisor reset robot to start position")

'''
supervisor_get_enemy_positions - supervisor utility function that returns the 
current position of the enemies (epucks) on the course
@returns - list of coordinates
'''
def supervisor_get_enemy_positions():
    global supervisor
    coords_list = []

    root_children_field = supervisor.getRoot().getField("children") 
    for idx in range(root_children_field.getCount()):
        if root_children_field.getMFNode(idx).getDef() == "Enemy":
            enemy_node = root_children_field.getMFNode(idx)
            enemy_coords = enemy_node.getField("translation").getSFVec3f()
            coords_list.append(np.array([enemy_coords[0], enemy_coords[2]]))

    return coords_list

'''
supervisor_get_walls - supervisor utility function that can return the position of the walls
in the course
@returns - list of coordinates, dimensions, and rotations
'''
def supervisor_get_walls():
    global supervisor
    coords_list = []

    root_children_field = supervisor.getRoot().getField("children") 
    for idx in range(root_children_field.getCount()):
        if root_children_field.getMFNode(idx).getDef() == "Wall":
            wall_node = root_children_field.getMFNode(idx)
            wall_coords = wall_node.getField("translation").getSFVec3f()
            wall_rot = wall_node.getField("rotation").getSFRotation()
            # I'd love to change this line if possible
            wall_dimens = wall_node.getField("children").getMFNode(0).getField("geometry").getSFNode().getField("size").getSFVec3f()
            coords_list.append(np.array([wall_coords[0], wall_coords[2], wall_dimens[0], wall_dimens[2], wall_rot[-1]]))

    return coords_list

def supervisor_get_target_pose():
    '''
    Returns target position relative to the robot's current position.
    Do not call during your solution! Only during problem setup and for debugging!
    '''

    target_position = np.array(target_node.getField("translation").getSFVec3f())
    target_pose = np.array([target_position[0], 1. - target_position[2], target_node.getField("rotation").getSFRotation()[3] + math.pi/2.])
    # print("Target pose relative to robot: %s" % (str(target_pose)))
    return target_pose

    
def supervisor_get_robot_pose():
    """
    Returns robot position
    """
    robot_position = np.array(robot_node.getField("translation").getSFVec3f())
    robot_pose = np.array([robot_position[0], 1. - robot_position[2], robot_node.getField("rotation").getSFRotation()[3]+math.pi/2])
    return robot_pose