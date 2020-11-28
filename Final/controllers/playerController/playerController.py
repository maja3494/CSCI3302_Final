"""robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
from controller import Robot, Motor, DistanceSensor, GPS
import playerSupervisor
import numpy as np

# init the supervisor
playerSupervisor.init_supervisor()
robot = playerSupervisor.supervisor

'''
Global variables for world state
'''
ENEMY_COORDS = playerSupervisor.supervisor_get_enemy_positions()
WALL_COORDS = playerSupervisor.supervisor_get_walls()


'''
Global variables concerning the EPUCK
'''
EPUCK_DIAMETER = .074
EPUCK_RADIUS = EPUCK_DIAMETER / 2

'''
Global variables being analagous to MACROs
'''
WHEEL_FORWARD = 1
WHEEL_STOPPED = 0
WHEEL_BACKWARD = -1
SIM_TIMESTEP = int(robot.getBasicTimeStep())


leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)


'''
check_point_v_enemy - checks if a point is valid relative to the obstacle epucks.
Does factor in the width of both epucks.
@param: p - a list-like x, y pair in world coordinates
@return: True if the point is valid False otherwise
'''
def check_point_v_enemy(p):
    global ENEMY_COORDS
    ENEMY_COORDS = playerSupervisor.supervisor_get_enemy_positions()

    for coord in ENEMY_COORDS:
        if (np.linalg.norm(coord - np.array(p)) < EPUCK_DIAMETER):
            return False

    return True

'''
check_point_v_walls - checks if a point is valid relative to the obstacle epucks.
Does factor in the width of both epucks.
@param: p - a list-like x, y pair in world coordinates
@return: True if the point is valid False otherwise
'''
def check_point_v_walls(p):
    global WALL_COORDS

    for coord in WALL_COORDS:
        coord_point = coord[0:2]
        coord_dimen = np.array([coord[2], coord[3]]) if coord[4] == 0 else np.array([coord[3], coord[2]])
        # TODO: this could be reused to prevent recalculation
        lower_bound_x = coord_point[0] - (coord_dimen[0] / 2) - EPUCK_DIAMETER
        upper_bound_x = coord_point[0] + (coord_dimen[0] / 2) + EPUCK_DIAMETER
        lower_bound_y = coord_point[1] - (coord_dimen[1] / 2) - EPUCK_DIAMETER
        upper_bound_y = coord_point[1] + (coord_dimen[1] / 2) + EPUCK_DIAMETER

        if (p[0] >= lower_bound_x and p[0] <= upper_bound_x and p[1] >= lower_bound_y and p[1] <= upper_bound_y):
            return False

    return True

def main():
    global ENEMY_COORDS

    # sensor burn in period
    for i in range(10): robot.step(SIM_TIMESTEP)
    
    # while robot.step(SIM_TIMESTEP) != -1:
    print(check_point_v_walls((.43 - EPUCK_DIAMETER - .01,-.16)))
        
    
if __name__ == "__main__":
    main()





