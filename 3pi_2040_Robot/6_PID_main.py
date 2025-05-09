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

# def drive_straight(d):
#     """
#     We can wrap up the driving in a function to shorten syntax
#     Note: for some reason we need to press c a bunch of times to stop movement
#     """
#     controller.drive_straight(d)
#     wait_until_done(controller)
#     if interrupted:
#         controller.handle_interrupt()
#         return

while True:
    if button_a.check():
        while button_a.check(): pass  # wait for release
        time.sleep(0.25)
        controller.handle_interrupt()  # Reset screen before starting new sequence

        # === Student-controlled motion sequence ===
        controller.drive_straight(1000)
        wait_until_done(controller)
        if interrupted:
            controller.handle_interrupt()
            continue

        controller.turn_degrees(90)
        wait_until_done(controller)
        if interrupted:
            controller.handle_interrupt()
            continue

        controller.drive_straight(5000)
        wait_until_done(controller)
        if interrupted:
            controller.handle_interrupt()
            continue

        controller.turn_degrees(-45)
        wait_until_done(controller)
        if interrupted:
            controller.handle_interrupt()
            continue

        controller.drive_straight(2000)
        wait_until_done(controller)
        if interrupted:
            controller.handle_interrupt()
            continue

        controller.turn_degrees(-90)
        wait_until_done(controller)
        if interrupted:
            controller.handle_interrupt()
            continue

        controller.drive_straight(5000)
        wait_until_done(controller)
        if interrupted:
            controller.handle_interrupt()
            continue



