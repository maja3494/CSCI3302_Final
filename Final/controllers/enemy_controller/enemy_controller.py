"""enemy_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

#GLOBALS
WHEEL_FORWARD = 1
WHEEL_STOPPED = 0
WHEEL_BACKWARD = -1
TimeTillTurn = 165

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# initialize motor variables
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

state = "stationary" #change to moving or stationary
sub_state = "forward"

# Main loop:
def main():
    # - perform simulation steps until Webots is stopping the controller
    count = 0
    while robot.step(timestep) != -1:
        if(state == "stationary"):
            pass
        elif(state == "moving"):
            if(sub_state == "forward"):
                velocity = WHEEL_FORWARD*leftMotor.getMaxVelocity()
                count += 1
                if(count == TimeTillTurn):
                    sub_state = "backward"
                    count = 0
            elif(sub_state == "backward"):
                velocity = WHEEL_BACKWARD*leftMotor.getMaxVelocity()
                count += 1
                if(count == TimeTillTurn):
                    sub_state = "forward"
                    count = 0
                
            leftMotor.setVelocity(velocity)
            rightMotor.setVelocity(velocity)
    # Any code to cleanup on exit

if __name__ == "__main__":
    main()