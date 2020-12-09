"""robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
from controller import Robot, Motor, DistanceSensor, GPS
import playerSupervisor
import numpy as np
from matplotlib import pyplot as plt

# init the supervisor
playerSupervisor.init_supervisor()
robot = playerSupervisor.supervisor

'''
Global variables for world state
'''
ENEMY_COORDS = playerSupervisor.supervisor_get_enemy_positions()
WALL_COORDS = playerSupervisor.supervisor_get_walls()
player_node = playerSupervisor.supervisor_get_robot_pose()
goal_node = playerSupervisor.supervisor_get_target_pose()

state = "get_path"
# the index of the waypoint along the path we are going to next
goal_waypoint = 0
# the path to the goal that we are following, filled in during state "get_path"
global_path = []
'''
Global variables concerning the EPUCK
'''
EPUCK_MAX_WHEEL_SPEED = 0.12880519 # m/s
EPUCK_DIAMETER = .074
EPUCK_RADIUS = EPUCK_DIAMETER / 2
EPUCK_AXLE_DIAMETER = 0.053
EPUCK_WHEEL_RADIUS = 0.0205

# GAIN Values
theta_gain = 1.0
distance_gain = 0.3

MAX_VEL_REDUCTION = 0.5

# Robot Pose Values
pose_x = 0
pose_y = 0
pose_theta = 0
left_wheel_direction = 0
right_wheel_direction = 0

'''
Global variables being analagous to MACROs
'''
WHEEL_FORWARD = 1
WHEEL_STOPPED = 0
WHEEL_BACKWARD = -1
SIM_TIMESTEP = int(robot.getBasicTimeStep())
BOUNDS = np.array([[-.4, .43], [-.48, .54]])
ENEMY_TIME_TO_TURN = 165
ENEMY_POS = np.zeros((4, ENEMY_TIME_TO_TURN))

# the extra distance we want to leave between an epuck and an enemy
# an epuck is .074 diameter, so this is about a quarter of an epuck
COLLISION_BUFFER = .02

leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)


def translate_y(y):
    '''
    gets the y compatible with our lab implementations
    @param - a world y value
    '''
    return BOUNDS[1][1] - y

def update_odometry(left_wheel_direction, right_wheel_direction, time_elapsed):
    '''
    Given the amount of time passed and the direction each wheel was rotating,
    update the robot's pose information accordingly
    '''
    global pose_x, pose_y, pose_theta, EPUCK_MAX_WHEEL_SPEED, EPUCK_DIAMETER
    pose_theta += (right_wheel_direction - left_wheel_direction) * time_elapsed * EPUCK_MAX_WHEEL_SPEED / EPUCK_DIAMETER
    pose_x += math.cos(pose_theta) * time_elapsed * EPUCK_MAX_WHEEL_SPEED * (left_wheel_direction + right_wheel_direction)/2.
    pose_y += math.sin(pose_theta) * time_elapsed * EPUCK_MAX_WHEEL_SPEED * (left_wheel_direction + right_wheel_direction)/2.
    pose_theta = get_bounded_theta(pose_theta)

def get_bounded_theta(theta):
    '''
    Returns theta bounded in [-PI, PI]
    '''
    while theta > math.pi: theta -= 2.*math.pi
    while theta < -math.pi: theta += 2.*math.pi
    return theta

def get_wheel_speeds(target_pose):
    '''
    @param target_pose: Array of (x,y,theta) for the destination robot pose
    @return motor speed as percentage of maximum for left and right wheel motors
    '''
    global pose_x, pose_y, pose_theta, left_wheel_direction, right_wheel_direction

    pose_x, pose_y, pose_theta = playerSupervisor.supervisor_get_robot_pose()

    pose_y = translate_y(pose_y)

    bearing_error = get_bounded_theta(math.atan2( (target_pose[1] - pose_y), (target_pose[0] - pose_x) ) - pose_theta)
    distance_error = np.linalg.norm(target_pose[:2] - np.array([pose_x,pose_y]))
    heading_error = get_bounded_theta(target_pose[2] -  pose_theta)

    BEAR_THRESHOLD = 0.06
    DIST_THRESHOLD = 0.03
    dT_gain = theta_gain
    dX_gain = distance_gain
    if distance_error > DIST_THRESHOLD:
        # print("dist error is large: fixing bearing, error is:", bearing_error)
        dTheta = bearing_error
        if abs(bearing_error) > BEAR_THRESHOLD:
            dX_gain = 0
    else:
        # print("fixing heading, error is:", heading_error)
        dTheta = heading_error
        dX_gain = 0

    dTheta *= dT_gain
    dX = dX_gain * min(3.14159, distance_error)

    phi_l = (dX - (dTheta*EPUCK_AXLE_DIAMETER/2.)) / EPUCK_WHEEL_RADIUS
    phi_r = (dX + (dTheta*EPUCK_AXLE_DIAMETER/2.)) / EPUCK_WHEEL_RADIUS
    
    wheel_rotation_normalizer = max(abs(phi_l), abs(phi_r))
    left_speed_pct = (phi_l) / wheel_rotation_normalizer
    right_speed_pct = (phi_r) / wheel_rotation_normalizer
    
    # we'll deal with stopping ourselves if we're close enough,
    # so set these to be super low
    # we switch waypoints at .03 distance for reference
    if distance_error < 0.01 and abs(heading_error) < 0.05:
        # if distance_error < .01:
            # print("setting speeds to 0 since distance is low:", distance_error)
        # if abs(heading_error) < .05:
            # print("setting speeds to 0 since heading is low:", heading_error)
        left_speed_pct = 0
        right_speed_pct = 0
        
    left_wheel_direction = left_speed_pct * MAX_VEL_REDUCTION
    phi_l_pct = left_speed_pct * MAX_VEL_REDUCTION * leftMotor.getMaxVelocity()

    right_wheel_direction = right_speed_pct * MAX_VEL_REDUCTION
    phi_r_pct = right_speed_pct * MAX_VEL_REDUCTION * rightMotor.getMaxVelocity()
       
    return phi_l_pct, phi_r_pct

'''
RRT Stuff
'''
class Node:
    """
    Node for RRT/RRT* Algorithm
    """
    def __init__(self, pt, parent=None):
        self.point = pt # n-Dimensional point
        self.parent = parent # Parent node
        self.path_from_parent = [] # List of points along the way from the parent node (for visualization)

'''
get_random_valid - get a new point in the map
'''
def get_random_valid_vertex():
    global BOUNDS
    vertex = None
    while vertex is None: # Get starting vertex
        pt = np.random.rand(2) * (BOUNDS[:,1]-BOUNDS[:,0]) + BOUNDS[:,0]
        if check_point_v_enemy(pt) and check_point_v_walls(pt):
            vertex = pt
    return vertex

def get_nearest_vertex(node_list, q_point):
    '''
    @param node_list: List of Node objects
    @param q_point: n-dimensional array representing a point
    @return Node in node_list with closest node.point to query q_point
    '''
    neighbor = None
    min_dist = float('Inf')

    for node in node_list:
        # print(node.point, q_point)
        dist = np.linalg.norm(node.point - q_point)

        if (dist < min_dist):
            neighbor = node
            min_dist = dist

    return neighbor

def steer(from_point, to_point, delta_q):
    '''
    @param from_point: n-Dimensional array (point) where the path to "to_point" is originating from (e.g., [1.,2.])
    @param to_point: n-Dimensional array (point) indicating destination (e.g., [0., 0.])
    @param delta_q: Max path-length to cover, possibly resulting in changes to "to_point" (e.g., 0.2)
    @returns path: list of points leading from "from_point" to "to_point" (inclusive of endpoints)  (e.g., [ [1.,2.], [1., 1.], [0., 0.] ])
    '''

    q_new  = to_point if (np.linalg.norm(to_point - from_point) <= delta_q) else from_point + ((to_point - from_point) * delta_q / np.linalg.norm(to_point - from_point))
    path = np.linspace(from_point, q_new, 10)

    return path, q_new

def rrt(starting_point, goal_point, k, delta_q):
    '''
    @param starting_point: Point within state_bounds to grow the RRT from
    @param goal_point: Point within state_bounds to target with the RRT. (OPTIONAL, can be None)
    @param k: Number of points to sample
    @param delta_q: Maximum distance allowed between vertices
    @returns List of RRT graph nodes
    '''

    node_list = []

    node_list.append(Node(starting_point, parent=None)) # Add Node at starting point with no parent
    node_list[0].path_from_parent += [node_list[0].point]

    for i in range(k):
        valid = False
        while (not valid):
            # Naively sample a point (and is there is a goal state periodically use that)
            new_point = get_random_valid_vertex() if (goal_point is None or np.random.random_sample() > .05) else goal_point
            # Get this closest parent to this point
            parent = get_nearest_vertex(node_list, new_point)

            # Scale this to proper distance
            path, new_point = steer(parent.point, new_point, delta_q)

            # Check validity of point
            valid = True
            for p in path:
                valid = valid and check_point_v_enemy(p) and check_point_v_walls(p)

        # Add node to tree
        new_node = Node(new_point, parent=parent)
        new_node.path_from_parent = new_node.parent.path_from_parent.copy()
        new_node.path_from_parent += [new_node.point]
        node_list.append(new_node)

        if (np.array_equal(new_point, goal_point)):
            break

    return node_list

def visualize_2D_graph(nodes, goal_point=None, wait_for_close = True):
    '''
    @param nodes: List of vertex locations
    @param goal_point: Where to draw the goal at
    @param wait_for_close: Whether the program should wait for the user to close the chart window before continuing to run
                                I highly suggest waiting, as it is incredibly buggy and weird otherwise
    '''
    global BOUNDS


    goal = None
    for node in nodes:
        if node.parent is not None:
            node_path = np.array(node.path_from_parent)
            plt.plot(node_path[:,0], translate_y(node_path[:,1]), '-b')
        if goal_point is not None and np.linalg.norm(node.point - np.array(goal_point)) <= 1e-5:
            goal = node
            plt.plot(node.point[0], translate_y(node.point[1]), 'k^')
        else:
            plt.plot(node.point[0], translate_y(node.point[1]), 'ro')

    plt.plot(nodes[0].point[0], translate_y(nodes[0].point[1]), 'ko')

    if goal is not None:
        cur_node = goal
        while cur_node is not None:
            if cur_node.parent is not None:
                node_path = np.array(cur_node.path_from_parent)
                plt.plot(node_path[:,0], translate_y(node_path[:,1]), '--y')
                cur_node = cur_node.parent
            else:
                break

    if goal_point is not None:
        plt.plot(goal_point[0], translate_y(goal_point[1]), 'gx')

    if wait_for_close:
        plt.show()
    else:
        plt.draw()
        plt.pause(10**-3)

def burn_in_sensors():
    """
    give the sensors a chance to burn in
    """
    for _ in range(10): robot.step(int(SIM_TIMESTEP))

def check_point_v_enemy(p):
    '''
    check_point_v_enemy - checks if a point is valid relative to the obstacle epucks.
    Does factor in the width of both epucks.
    @param: p - a list-like x, y pair in world coordinates
    @return: True if the point is valid False otherwise
    '''
    global ENEMY_COORDS
    ENEMY_COORDS = playerSupervisor.supervisor_get_enemy_positions()

    for coord in ENEMY_COORDS:
        if (np.linalg.norm(coord - np.array(p)) < EPUCK_DIAMETER):
            return False

    return True

def check_point_v_walls(p):
    '''
    check_point_v_walls - checks if a point is valid relative to the obstacle epucks.
    Does factor in the width of both epucks.
    @param: p - a list-like x, y pair in world coordinates
    @return: True if the point is valid False otherwise
    '''
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

def initEnemyCoords():
    global ENEMY_COORDS, ENEMY_POS
    ENEMY_COORDS = playerSupervisor.supervisor_get_enemy_positions()

    disp = 0.349147 - ENEMY_COORDS[0][0]

    for i in range(len(ENEMY_COORDS)):
        x = ENEMY_COORDS[i][0]
        if x > 0:
            ENEMY_POS[i, :] = np.linspace(x, x - disp, ENEMY_TIME_TO_TURN)
        else:
            ENEMY_POS[i, :] = np.linspace(x, x + disp, ENEMY_TIME_TO_TURN)

def getEnemyPos(time):
    '''
    Matt
    Gets epuck position at given time
    @param time - timestep of interest
    @return list of x, y coordinates
    '''
    index = None
    direction = int(time / ENEMY_TIME_TO_TURN) % 2
    index = time % ENEMY_TIME_TO_TURN
    if direction:
        index = ENEMY_TIME_TO_TURN - index
    curr_index =[]
    for i in range(4):
        curr_index.append([ENEMY_POS[i][index],ENEMY_COORDS[i][1]])
    return curr_index

def straightLine(coord1, coord2):
    """
    Ryan
    return # timesteps
    """
    distance = math.sqrt( (coord1[0]-coord2[0])**2 + (coord1[1]-coord2[1])**2 )

    timestepsTillAtGoal = ( distance / EPUCK_MAX_WHEEL_SPEED ) / .032

    return timestepsTillAtGoal

def turnTime(theta):
    '''
    Luke
    return # timesteps
    '''
    pass

def moving_enemy_collision(next_point, timestep_arrive):
    '''
    Jake
    @param: next_point - the point of the player to check at
    @param: timestep - the timestep to use to find the enemy position
    @return: bool, True if a collision occurs, false if no collision
    '''
    #get the positions of all of the enemies
    all_enemies_pos = getEnemyPos(timestep_arrive)
    # convert our player to a np list so we can use their np.linalg.norm
    next_as_np = np.array(next_point)
    # get the distances from each enemy to our player
    distances = [np.linalg.norm(next_as_np - np.array(enemy_pos)) for enemy_pos in all_enemies_pos]
    # check if any of the enemies are hitting the player
    # since they're both circles, their distance when touching will be enemy_rad + player_rad
    # since both are the same, I just used diameter
    is_collision_list = [dist <= EPUCK_DIAMETER + COLLISION_BUFFER for dist in distances]
    # return true if any of them are colliding, false otherwise
    return any(is_collision_list)

def calcAngles(from_point, to_point):
    '''
    Jake
    return angle we are trying to get to
    @param: from_point - the point that we as the starting point for our line
    @param: to_point - the point that we use as the ending point of our line
    @return: the angle from the positive x-axis coming out of the from-point, moving counterclockwise to our to-point
    '''
    #unwrap the points since it makes it easier to understand
    from_x, from_y = from_point
    to_x, to_y = to_point
    return np.arctan2(to_y - from_y, to_x - from_x)

def calcAnglesThree(from_point, curr_point, to_point):
    '''
    Jake
    @param: from_point - the oldest point in the path
    @param: curr_point - the middle point that we're turning on
    @param: to_point - the furthest point that we are heading to
    @return: the angle you need to turn counter-clockwise when transitioning from traveling between two lines
    angle between the lines from_point:curr_point and the line curr_point:to_point
    '''
    # get the original angle
    orig_angle = calcAngles(from_point, curr_point)
    # get the next angle
    next_angle = calcAngles(curr_point, to_point)
    # return how much you have to rotate
    return next_angle - orig_angle

def main():
    global player_node, goal_node, BOUNDS
    global robot, state, global_path, goal_waypoint
    global leftMotor, rightMotor, SIM_TIMESTEP, WHEEL_FORWARD, WHEEL_STOPPED, WHEEL_BACKWARD
    global pose_x, pose_y, pose_theta, left_wheel_direction, right_wheel_direction

    # last_odometry_update_time = None

    # define the init enemy coords
    initEnemyCoords()

    print(ENEMY_POS)
    # max number of nodes to include in RRT
    K = 250 # Feel free to adjust as desired

    while robot.step(SIM_TIMESTEP) != -1:

        # if last_odometry_update_time is None:
        #         last_odometry_update_time = robot.getTime()
        # time_elapsed = robot.getTime() - last_odometry_update_time
        # update_odometry(left_wheel_direction, right_wheel_direction, time_elapsed)
        # last_odometry_update_time = robot.getTime()

        if state == 'get_path':
            print("Computing the path")

            starting_point = player_node
            # Compute a path from start to target_pose
            print("goal:",goal_node[:2])
            psuedo_goal=goal_node[:2].copy()
            # TODO: Don't hardcode this
            psuedo_goal[0] += 0.2
            if check_point_v_walls(psuedo_goal):
                nodes = rrt(starting_point[:2], psuedo_goal, K, np.linalg.norm(BOUNDS/10.))
                # TODO: Doesn't work great after replan
                # visualize_2D_graph(nodes, psuedo_goal, wait_for_close = False)
                # loop through the rrt to find the goal point
                goal_index = -1
                for i in range(0,len(nodes)):
                    if np.linalg.norm(psuedo_goal-nodes[i].point) < 1e-5:
                        goal_index = i
                        break

                # did we ever end up finding the goal
                if goal_index != -1:
                    # the path to it is already stored in our "path_to_parent"
                    global_path = nodes[goal_index].path_from_parent.copy()
                    # just in case it only got close, just change the last item to the goal
                    global_path[-1] = goal_node[:2]
                    # start at the beginning
                    goal_waypoint = 0
                    state = 'get_waypoint'
                else:
                    print("Impossible path computed")
                    break
            else:
                print("Select a valid target position")
                break

        elif state == 'get_waypoint':
            print("Getting the next waypoint")
            # make sure we didn't somehow go out of bounds
            if goal_waypoint >= len(global_path) or goal_waypoint < 0:
                state='get_path'
                continue
            next_x, next_y = global_path[goal_waypoint]
            next_y = translate_y(next_y)

            # to determine the goal angle, we need to know whether this is the last node or not
            if goal_waypoint == len(global_path) - 1:
                # is the end point, just point wherever
                angle = 0
            else:
                # not the end point, point towards the next
                after_x, after_y = global_path[goal_waypoint+1]
                after_y = translate_y(after_y)
                angle = calcAngles((next_x, next_y), (after_x, after_y))
            waypoint = (next_x, next_y, angle)
            print("Moving to the next waypoint:", waypoint)
            state = 'move_to_waypoint'

        elif state == 'move_to_waypoint':
            #get wheel speed heading error currently set to 1 we will need to change that eventually.
            lspeed, rspeed = get_wheel_speeds(waypoint)
            leftMotor.setVelocity(lspeed)
            rightMotor.setVelocity(rspeed)

            dist_to_waypoint = np.linalg.norm(np.array([pose_x, pose_y]) - waypoint[:2])
            
            if dist_to_waypoint > .3:
                print("we're crazy far away, getting the path again")
                state = "get_path"
            elif dist_to_waypoint < .03:
                # try to get the next waypoint
                goal_waypoint += 1

                # but check if it's the last goal or not
                if goal_waypoint == len(global_path):
                    state = "arrived"
                else:
                    state = 'get_waypoint'
            # else: we just continue moving

        elif state == "arrived":
            print("Arrived at destination")
            # Stop
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break

        if (playerSupervisor.check_collisions() >= 5): # the 5 might break at some point 4 is too low though
            playerSupervisor.supervisor_reset_to_home()
            burn_in_sensors()
            state = "get_path"


if __name__ == "__main__":
    main()
