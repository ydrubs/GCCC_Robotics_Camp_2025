import time
from pololu_3pi_2040_robot import robot
from _PID_controller_ import TurnController, wait_until_done, interrupted


controller = TurnController()
display = robot.Display()
button_a = robot.ButtonA()

# Initial screen
display.fill(0)
display.text("A=start  C=stop", 0, 0, 1)
display.show()

def process_movement():
    wait_until_done(controller)
    if interrupted:
        controller.handle_interrupt()
        return

def move_sequence():
    # === Student-controlled motion sequence ===
    controller.drive_straight(1000)
    process_movement()

    controller.turn_degrees(90)
    process_movement()

    controller.drive_straight(5000)
    process_movement()

    controller.turn_degrees(-45)
    process_movement()

    controller.drive_straight(2000)
    process_movement()

    controller.turn_degrees(-90)
    process_movement()

    controller.drive_straight(5000)
    process_movement()

while True:
    if button_a.check():
        while button_a.check(): pass  # wait for release
        time.sleep(0.25)
        controller.handle_interrupt()  # Reset screen before starting new sequence

        move_sequence()



