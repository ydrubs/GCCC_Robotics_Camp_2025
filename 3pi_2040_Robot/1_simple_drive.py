from pololu_3pi_2040_robot import robot
import time

motors = robot.Motors()
speed = motors.MAX_SPEED #Defined as -6000 to 6000

while True:

    motors.set_speeds(speed,speed) # Allows for values from -6000 to 6000 per motor channel
    time.sleep(1)
    motors.set_speeds(0,speed)
    time.sleep(1)
    motors.set_speeds(-speed, -speed)
    time.sleep(1)
    motors.off()
    time.sleep(2)


