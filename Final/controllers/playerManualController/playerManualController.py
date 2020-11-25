"""playerManualController controller"""
"""
this controller allows for user control of an epuck
they can use either the arrow keys or wasd to control it
it supports combinations of keys, for up to a total of 8 directions and stopped
"""

import math
from controller import Robot, Motor, DistanceSensor, GPS
import numpy as np

# reference to the Keyboard module:
# https://www.cyberbotics.com/doc/reference/keyboard?tab-language=python
from controller import Keyboard
keyboard = Keyboard()

WHEEL_FORWARD = 1
WHEEL_STOPPED = 0
WHEEL_BACKWARD = -1

robot = Robot()
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# what speed to have the epuck drive at
MAX_VEL_REDUCTION = 1 # player will want to be fast as possible
LEFT_VEL_REDUCTION = MAX_VEL_REDUCTION * leftMotor.getMaxVelocity()
RIGHT_VEL_REDUCTION = MAX_VEL_REDUCTION * rightMotor.getMaxVelocity()

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# exchanges keyboard keys for numeric directions
# see function getDirection() for encodings
# allows for both keypad and wasd
key2dir = {
    Keyboard.LEFT: 6,
    65: 6, # a
    Keyboard.RIGHT: 2,
    68: 2, # d
    Keyboard.UP: 0,
    87: 0, # w
    Keyboard.DOWN: 4,
    83: 4 # s
}

dir2wheels = {
    0: (WHEEL_FORWARD, WHEEL_FORWARD),
    1: (WHEEL_FORWARD, .5 * WHEEL_FORWARD),
    2: (WHEEL_FORWARD, WHEEL_BACKWARD),
    3: (WHEEL_BACKWARD, WHEEL_STOPPED), # yes, forward and backwards angles act different :|
    4: (WHEEL_BACKWARD, WHEEL_BACKWARD),
    5: (WHEEL_STOPPED, WHEEL_BACKWARD),
    6: (WHEEL_BACKWARD, WHEEL_FORWARD),
    7: (.5 * WHEEL_FORWARD, WHEEL_FORWARD),
    -1: (WHEEL_STOPPED, WHEEL_STOPPED)
}


# reads the keys and converts them into a direction to travel in
# values range from 0-7 and -1, representing the 8 directions and stopped
# 0 is forward and they increase clockwise, with 2 being right, 4 being
# backwards, 6 being left, and -1 being stopped
def getDirection():
    global keyboard

    # -gets the current key pressed, else -1 for no keys
    key1 = keyboard.getKey()
    # if no key or invalid key, just stop
    if key1 == -1 or key1 not in key2dir:
        return -1
    
    # calling it again gets the second key, else -1 if no second key
    key2 = keyboard.getKey()
    # if no key or not valid, just use first only
    if key2 == -1 or key2 not in key2dir:
        return key2dir[key1]
    
    # if they're close, merge the two keys together to get a joint direction
    key1 = key2dir[key1]
    key2 = key2dir[key2]
    # opposite keys/across from each other, just do the first one
    if abs(key1 - key2) == 4:
        return key1
    # 0 and 6, the one special case since it loops
    if abs(key1 - key2) == 6:
        return 7
    # otherwise, just average them
    return (key1 + key2) // 2



def main():
    global keyboard

    # enable the keyboard to start getting inputs
    # the argument is the sampling period, in milliseconds
    # I put the timestep because it feels right, this may need fixing later
    keyboard.enable(SIM_TIMESTEP)
    
    # try/except so that we can stop the robot if anything strange happens
    #   this is mostly since the user has control here and so we don't know what to expect
    #   and also because I was breaking it a lot earlier
    try:
        while robot.step(SIM_TIMESTEP) != -1:
            direction = getDirection()
            (left_dir, right_dir) = dir2wheels[direction]

            leftMotor.setVelocity(left_dir * LEFT_VEL_REDUCTION)
            rightMotor.setVelocity(right_dir * RIGHT_VEL_REDUCTION)

    # stop the robot whether we ran into an error or not
    finally:
        leftMotor.setVelocity(WHEEL_STOPPED * leftMotor.getMaxVelocity())
        rightMotor.setVelocity(WHEEL_STOPPED * rightMotor.getMaxVelocity())
    
if __name__ == "__main__":
    main()





