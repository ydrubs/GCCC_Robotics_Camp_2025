from pololu_3pi_2040_robot import robot
import time

encoders = robot.Encoders()
motors = robot.Motors()
display = robot.Display()
buttonB = robot.ButtonB()

def move_distance():
    motors.set_speeds(motors.MAX_SPEED, motors.MAX_SPEED)

while True:
    c = encoders.get_counts()

    display.fill_rect(0, 0, 128, 18, 0)  # Clear the top portion of the screen (128x18 pixels) by filling it with black (color 0)
    display.text("Left: " + str(c[0]), 0, 0)  # Draw the left encoder count as text at coordinates (0, 0)
    display.text("Right: " + str(c[1]), 0, 10)  # Draw the right encoder count as text at coordinates (0, 10), slightly lower on the screen
    display.show()  # Refresh the screen to display the updated text and graphics

    if buttonB.check():
        encoders.get_counts(reset = True)
        time.sleep(0.5)

    if c[0] < 1000 or c[1] < 1000:
        move_distance()

    else:
        motors.off()