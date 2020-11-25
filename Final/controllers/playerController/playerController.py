"""robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
from controller import Robot, Motor, DistanceSensor, GPS
import numpy as np

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

def main():
    leftMotor.setVelocity(WHEEL_STOPPED * leftMotor.getMaxVelocity())
    rightMotor.setVelocity(WHEEL_STOPPED * rightMotor.getMaxVelocity())
    
if __name__ == "__main__":
    main()





