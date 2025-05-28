from pololu_3pi_2040_robot import robot
import time

# Setup hardware
buzzer = robot.Buzzer()
button_c = robot.ButtonC()
motors = robot.Motors()
rgb_leds = robot.RGBLEDs()
rgb_leds.set_brightness(5)

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

# LED colors to rotate through
colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255],
          [128, 0, 128], [255, 255, 0], [255, 255, 255]]


def blink_all_leds(step_count):
    """Cycle all 6 LEDs with an offset pattern."""
    for i in range(6):
        rgb_leds.set(i, colors[(step_count + i) % len(colors)])
    rgb_leds.show()


def timed_dance_move(left_speed, right_speed, step_count, duration_ms=1000):
    """Run a motor move for a duration, blinking LEDs rapidly during it."""
    motors.set_speeds(left_speed, right_speed)

    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
        blink_all_leds(step_count)
        time.sleep(0.1)  # Blink LEDs every 0.1 seconds


def dance_step(step_count):
    """Perform a sequence of motor moves with fast LED blinks."""
    timed_dance_move(motors.MAX_SPEED, -motors.MAX_SPEED, step_count)
    timed_dance_move(-motors.MAX_SPEED / 2, motors.MAX_SPEED / 2, step_count + 1)
    timed_dance_move(motors.MAX_SPEED // 2, motors.MAX_SPEED // 4, step_count + 2)
    timed_dance_move(motors.MAX_SPEED // 4, motors.MAX_SPEED // 2, step_count + 3)
    timed_dance_move(-motors.MAX_SPEED // 2, -motors.MAX_SPEED // 4, step_count + 4)
    timed_dance_move(-motors.MAX_SPEED // 4, -motors.MAX_SPEED // 2, step_count + 5)


def dance_while_music():
    buzzer.play_in_background(mario_theme)
    step_counter = 0

    while buzzer.is_playing():
        dance_step(step_counter)
        step_counter += 6

    motors.off()
    buzzer.off()

    for i in range(6):
        rgb_leds.set(i, [0, 0, 0])
    rgb_leds.show()


# Start dance routine when Button C is pressed
while True:
    if button_c.check() and button_c.is_pressed():
        dance_while_music()
