from pololu_3pi_2040_robot import robot
import time

buzzer = robot.Buzzer()

button_a = robot.ButtonA()
button_b = robot.ButtonB()
button_c = robot.ButtonC()
motors = robot.Motors()
buzzer = robot.Buzzer()

# Define the Theme Song
mario_theme = (
    "t200 l8 v15 ms "
    "O5 e e r e r c e r g r G r "
    "O4 g r r O5 c r r O4 g r r "
    "O5 e r e r e r c e r g r G r "
    "O4 g r r O5 c r r O4 g r r "

    "O5 g a g e c d b r "
    "O4 g r O5 e r a r b r "
    "O4 a r O5 c r r g r G r "
)



def dance_step():
    motors.set_speeds(motors.MAX_SPEED, -motors.MAX_SPEED)
    time.sleep(1)
    motors.set_speeds(-motors.MAX_SPEED/2, motors.MAX_SPEED/2)
    time.sleep(1)


    motors.set_speeds(motors.MAX_SPEED // 2, motors.MAX_SPEED // 4)
    time.sleep(1)
    motors.set_speeds(motors.MAX_SPEED // 4, motors.MAX_SPEED // 2)
    time.sleep(1)

    motors.set_speeds(-motors.MAX_SPEED // 2, -motors.MAX_SPEED // 4)
    time.sleep(1)
    motors.set_speeds(-motors.MAX_SPEED // 4, -motors.MAX_SPEED // 2)
    time.sleep(1)


def dance_while_music():
    buzzer.play_in_background(mario_theme)

    # Loop dance steps while song is playing
    while buzzer.is_playing():
        dance_step()

    motors.off()
    buzzer.off()

while True:
    if button_c.check() and button_c.is_pressed():
        dance_while_music()
