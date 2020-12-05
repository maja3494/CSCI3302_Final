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
sub_state=0
global_path=[]
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

MAX_VEL_REDUCTION = 0.25

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

leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

'''
translate_y - gets the y to compatible with our lab implementations
@param - a world y value
'''
def translate_y(y):
    return (y - BOUNDS[1][1]) * -1

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

    # print(target_pose[2],pose_theta)
    bearing_error = math.atan2( (target_pose[1] - pose_y), (target_pose[0] - pose_x) ) - pose_theta
    distance_error = np.linalg.norm(target_pose[:2] - np.array([pose_x,pose_y]))
    heading_error = target_pose[2] -  pose_theta

    # print("h:",heading_error)

    BEAR_THRESHOLD = 0.06
    DIST_THRESHOLD = 0.03
    dT_gain = theta_gain
    dX_gain = distance_gain
    if distance_error > DIST_THRESHOLD:
        dTheta = bearing_error
        if abs(bearing_error) > BEAR_THRESHOLD:
            dX_gain = 0
    else:
        dTheta = heading_error
        dX_gain = 0

    dTheta *= dT_gain
    dX = dX_gain * min(3.14159, distance_error)

    phi_l = (dX - (dTheta*EPUCK_AXLE_DIAMETER/2.)) / EPUCK_WHEEL_RADIUS
    phi_r = (dX + (dTheta*EPUCK_AXLE_DIAMETER/2.)) / EPUCK_WHEEL_RADIUS

    left_speed_pct = 0
    right_speed_pct = 0
    
    wheel_rotation_normalizer = max(abs(phi_l), abs(phi_r))
    left_speed_pct = (phi_l) / wheel_rotation_normalizer
    right_speed_pct = (phi_r) / wheel_rotation_normalizer
    
    if distance_error < 0.05 and abs(heading_error) < 1:    
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
    @param state_bounds: matrix of min/max values for each dimension (e.g., [[0,1],[0,1]] for a 2D 1m by 1m square)
    @param state_is_valid: function that maps states (N-dimensional Real vectors) to a Boolean (indicating free vs. forbidden space)
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

def visualize_2D_graph(nodes, goal_point=None):
    '''
    @param state_bounds Array of min/max for each dimension
    @param obstacles Locations and radii of spheroid obstacles
    @param nodes List of vertex locations
    @param edges List of vertex connections
    '''
    global BOUNDS
    state_bounds = BOUNDS
    fig = plt.figure()
    plt.xlim(state_bounds[0,0], state_bounds[0,1])
    plt.ylim(state_bounds[1,0], state_bounds[1,1])


    goal = None
    for node in nodes:
        if node.parent is not None:
            node_path = np.array(node.path_from_parent)
            plt.plot(node_path[:,0], node_path[:,1], '-b')
        if goal_point is not None and np.linalg.norm(node.point - np.array(goal_point)) <= 1e-5:
            goal = node
            plt.plot(node.point[0], node.point[1], 'k^')
        else:
            plt.plot(node.point[0], node.point[1], 'ro')

    plt.plot(nodes[0].point[0], nodes[0].point[1], 'ko')

    if goal is not None:
        cur_node = goal
        while cur_node is not None:
            if cur_node.parent is not None:
                node_path = np.array(cur_node.path_from_parent)
                plt.plot(node_path[:,0], node_path[:,1], '--y')
                cur_node = cur_node.parent
            else:
                break

    if goal_point is not None:
        plt.plot(goal_point[0], goal_point[1], 'gx')

    plt.draw()
    plt.pause(10**-3)


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
    global player_node, goal_node, BOUNDS
    global robot, state, global_path, sub_state
    global leftMotor, rightMotor, SIM_TIMESTEP, WHEEL_FORWARD, WHEEL_STOPPED, WHEEL_BACKWARD
    # global pose_x, pose_y, pose_theta, left_wheel_direction, right_wheel_direction


    last_odometry_update_time = None

    # Keep track of which direction each wheel is turning
    left_wheel_direction = WHEEL_STOPPED
    right_wheel_direction = WHEEL_STOPPED


    # sensor burn in period
    for i in range(10): robot.step(SIM_TIMESTEP)

    K = 250 # Feel free to adjust as desired

    while robot.step(SIM_TIMESTEP) != -1:

        # if last_odometry_update_time is None:
        #         last_odometry_update_time = robot.getTime()
        # time_elapsed = robot.getTime() - last_odometry_update_time
        # update_odometry(left_wheel_direction, right_wheel_direction, time_elapsed)
        # last_odometry_update_time = robot.getTime()

        if state == 'get_path':
            sub_state=0
            starting_point = player_node
            # Compute a path from start to target_pose
            print("goal:",goal_node[:2])
            psuedo_goal=goal_node[:2].copy()
            psuedo_goal[0] += 0.2
            if check_point_v_walls(psuedo_goal):
                nodes = rrt(starting_point[:2], psuedo_goal, K, np.linalg.norm(BOUNDS/10.))
                visualize_2D_graph(nodes, psuedo_goal)
                valid_start = False
                valid_goal = -1
                for i in range(0,len(nodes)):
                    if np.linalg.norm(starting_point[:2]-nodes[i].point) < 1e-5:
                        valid_start = True
                    if np.linalg.norm(psuedo_goal-nodes[i].point) < 1e-5:
                        valid_goal = i
                if valid_goal != -1 and valid_start:
                    state = 'get_waypoint'
                    global_path=nodes[valid_goal].path_from_parent[:-1]
                    global_path.append(goal_node[:2])
                    sub_state=0
                else:
                    print("Impossible path computed")
                    break
            else:
                print("Select a valid target position")
                break

        elif state == 'get_waypoint':
            print("get new point:", )
            if len(global_path) - 1 <= sub_state:
                print("oops")
                state='get_path'
                continue
            curr=global_path[sub_state].copy()
            way_point=global_path[sub_state+1].copy()
            way_point[1] = translate_y(way_point[1])
            if sub_state + 1 == len(global_path) - 1:
                theta=0
            else:
                theta=math.atan2((way_point[1]-curr[1]),(way_point[0] - curr[0]))
            state='move_to_waypoint'
        elif state == 'move_to_waypoint':
            #get wheel speed heading error currently set to 1 we will need to change that eventually.
            lspeed, rspeed = get_wheel_speeds((way_point[0], way_point[1],0))
            leftMotor.setVelocity(lspeed)
            rightMotor.setVelocity(rspeed)

            if np.linalg.norm(goal_node[:2]-way_point) < 1e-5:
                state=="arrived"
                continue

            if lspeed==0 and rspeed==0:
                print("next state")
                state='get_waypoint'
                sub_state+=1

        elif state == "arrived":
                # Stop
                print('arrived')
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)


if __name__ == "__main__":
    main()
