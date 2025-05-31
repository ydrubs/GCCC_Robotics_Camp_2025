"""
Custom Dance Program

This program lets you create your own robot dance using simple move commands.
Each move controls the motors for a short time while music and LED lights play.

To create a custom move:

    - Use set_motors(left_speed, right_speed) to set how the robot moves.
    - Use run_move(duration_ms) to make it last for a certain time.
    - Use get_speed(speed) to calculate speed (0.1 = slow, 1.0 = full speed).

All your moves can be added in the dance_routine() function below.

Press Button C to start the show!
"""

from  _4a_robot_dance_CONTROLLER import DanceController

# === Mario Theme Song ===
"""
Copy/Pase the desired music from the file called '4b_music_for_robot_dance' 
If you change the name from maro_theme to something else, remember to also change the controller.set_music(<name>) reference
"""
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

# === Controller Instance ===
controller = DanceController()
controller.set_music(mario_theme)

# SET CUSTOM LED COLORS
controller.set_led_colors([
    [255, 0, 255],
    [0, 255, 255],
    [255, 255, 0],
    [0, 128, 255],
    [255, 128, 0],
    [255, 255, 255]
])

# === Student Helper Aliases ===
set_motors = controller.set_motors
run_move = controller.run_move
MAX_SPEED = controller.motors.MAX_SPEED

# === Utility Function ===
def get_speed(multiplier=1.0):
    multiplier = max(0.1, min(1.0, multiplier))
    return MAX_SPEED * multiplier

# === Move Functions (No Need for ctrl) ===
def pause(duration_ms=500):
    controller.motors.off()
    run_move(duration_ms)

def forward(duration_ms=600, speed=1.0):
    s = get_speed(speed)
    set_motors(s, s)
    run_move(duration_ms)

def backward(duration_ms=600, speed=1.0):
    s = get_speed(speed)
    set_motors(-s, -s)
    run_move(duration_ms)

def spin_left(duration_ms=600, speed=1.0):
    s = get_speed(speed)
    set_motors(-s, s)
    run_move(duration_ms)

def spin_right(duration_ms=600, speed=1.0):
    s = get_speed(speed)
    set_motors(s, -s)
    run_move(duration_ms)

def curve_right_gentle(duration_ms=600, speed=1.0):
    s = get_speed(speed)
    set_motors(s, s / 2)
    run_move(duration_ms)

def curve_left_gentle(duration_ms=600, speed=1.0):
    s = get_speed(speed)
    set_motors(s / 2, s)
    run_move(duration_ms)

def wiggle(cycles=4, interval_ms=150, speed=1.0):
    s = get_speed(speed)
    for _ in range(cycles):
        set_motors(-s, s)
        run_move(interval_ms)

        set_motors(s, -s)
        run_move(interval_ms)

    controller.motors.off()

def custom_move(duration_ms=600, speed=1.0):
    """Create Custom move with this function. Make sure to add the move name to the dance_routine function"""
    s = get_speed(speed)
    pass # Your custom movement here
    run_move(duration_ms)

# === Dance Routine Using Helpers ===
def dance_routine(ctrl):
    forward(600, 0.8)
    pause(500)

    spin_left(500, 0.6)
    pause(500)

    wiggle(cycles=3, interval_ms=320, speed=0.25)
    pause(500)

    curve_right_gentle(700, 0.9)
    pause(500)

    curve_left_gentle(700, 0.7)
    pause(500)

    spin_right(500, 0.8)
    pause(500)


# === Register and Wait for Button Press ===
controller.set_dance_routine(dance_routine)

while True:
    if controller.button_c.check() and controller.button_c.is_pressed():
        controller.run_once()
