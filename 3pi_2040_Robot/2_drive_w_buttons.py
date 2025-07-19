from pololu_3pi_2040_robot import robot
import time

button_a = robot.ButtonA() # Initialize Button A
button_b = robot.ButtonB() # Initialize Button B
motors = robot.Motors()

speed = motors.MAX_SPEED


while True:
    if button_a.check(): #Triggers when button is pressed
        time.sleep(0.5) # use so the robots doesn't lunge forward
        motors.set_speeds(speed/3,speed/3) # set to 2000


    if button_b.check():
        motors.off()

# noinspection PyUnreachableCode
"""
Tasks:
    When button A is pressed the robot moves in a square (or some other move sequene)
    When button B is pressed the robot moves in a circle (or some other sequence)
    When button C is pressed the motors turn off
"""