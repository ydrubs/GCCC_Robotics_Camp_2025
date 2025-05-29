"""
3pi+ Line Avoidance Robot — Calibration and Parameter Tuning Guide
==================================================================

This program drives the robot forward and performs a 180° turn when
its outer edge sensors detect strong black tape. It is designed to
stay within a boundary of dark lines, avoiding false triggers from
shadows, seams, or dim lighting.

How Calibration Works:
----------------------
1. Press Button A to start calibration.
2. The robot spins to expose its sensors to both light and dark surfaces.
3. Each sensor records its brightest (white) and darkest (black) readings.
4. These values are scaled so that:
   - 0 = brightest (e.g., white floor)
   - 1000 = darkest (e.g., black tape)

After calibration, the robot uses these scaled values to detect black lines.

How to Use:
-----------
1. Place the robot with some sensors over black tape and others over the floor.
2. Press Button A to calibrate.
3. After calibration is complete, press Button A again to begin driving.
4. The robot will move forward and turn 180° when it detects a black border.

Environment Tips:
-----------------
- For best results, use bright, evenly lit environments.
- Place tape on a white or reflective surface (paper, foam board).
- Avoid low light, shiny floors, or uneven backgrounds like wood grain.

Tuning Parameters:
------------------

border_threshold : int (typical range: 920–970)
    The minimum calibrated sensor value required to consider a surface "black enough."
    If a sensor reads above this threshold, it's considered to be seeing black tape.
    - Raise to reduce false triggers from shadows, seams, or wood grain.
    - Lower if the robot misses actual black lines.

strong_black_threshold : int (typical range: 950–990)
    A stricter threshold to confirm very dark black tape.
    At least one sensor must exceed this to trigger a turn.
    - Raise to avoid reacting to dark but non-tape surfaces.
    - Lower slightly if detection seems slow or misses tape.

confirm_required : int (typical: 1–2)
    Number of consecutive readings above the threshold required to confirm detection.
    - 1 = fastest reaction, may cause false turns
    - 2 = more stable, better in noisy environments or high-speed operation

drive_speed : int (typical: 1500–2500)
    Speed at which the robot drives forward.
    - Lower speeds give sensors more time to react and increase reliability.
    - Too fast may cause the robot to overshoot before detection.

turn_duration_ms : int (typical: 450–600 ms)
    Time the robot turns in place to reverse direction.
    - Tune this so that the robot completes a full 180° without overshooting or undershooting.

move_forward_after_turn_ms : int (typical: 300–600 ms)
    Time the robot moves forward after turning before rechecking for lines.
    - Prevents repeated turns if it’s still on or near the same line.
    - Too high may delay response to nearby new lines (e.g., in a corner).

Sensor Graph:
-------------
The display shows a live bar graph of the 5 line sensors:
- Tall bars = dark surface (e.g., black tape, low reflectance)
- Short bars = bright surface (e.g., white floor, high reflectance)

Use this graph during testing to visually verify line detection.
"""

from pololu_3pi_2040_robot import robot
import time

# === Hardware Setup ===
motors = robot.Motors()
line_sensors = robot.LineSensors()
button_a = robot.ButtonA()
display = robot.Display()

# === Settings ===
calibration_speed = 3000
calibration_count = 100
drive_speed = 2000
border_threshold = 940            # black must exceed this to count
strong_black_threshold = 970      # must also be very strong to confirm. SHOULD BE HIGHER THEN border_threshold
confirm_required = 1              # how many loops to confirm
turn_duration_ms = 550
move_forward_after_turn_ms = 400
graph_scale = 24 / 1000

# === Calibration Prompt ===
display.fill(0)
display.text("Place on line", 0, 0)
display.text("and press A to", 0, 10)
display.text("calibrate...", 0, 20)
display.show()

while not button_a.check():
    time.sleep_ms(10)

# === Sensor Calibration ===
display.fill(0)
display.text("Calibrating...", 0, 0)
display.show()

motors.set_speeds(calibration_speed, -calibration_speed)
for i in range(calibration_count // 4):
    line_sensors.calibrate()
motors.off()
time.sleep_ms(200)

motors.set_speeds(-calibration_speed, calibration_speed)
for i in range(calibration_count // 2):
    line_sensors.calibrate()
motors.off()
time.sleep_ms(200)

motors.set_speeds(calibration_speed, -calibration_speed)
for i in range(calibration_count // 4):
    line_sensors.calibrate()
motors.off()

# === Wait to Start Driving ===
display.fill(0)
display.text("Calibrated!", 0, 0)
display.text("Press A to start", 0, 20)
display.show()

while not button_a.check():
    time.sleep_ms(10)

# === Main Loop ===
display.fill(0)
display.text("Running...", 0, 0)
display.show()

motors.set_speeds(drive_speed, drive_speed)
confirm_counter = 0

while True:
    line = line_sensors.read_calibrated()
    line_sensors.start_read()

    # === Bar Graph Display ===
    display.fill_rect(0, 40, 128, 24, 0)
    for i in range(5):
        h = int(line[i] * graph_scale)
        display.fill_rect(36 + i * 12, 64 - h, 8, h, 1)

    display.text("Sensors", 0, 0)
    display.text(f"{line[0]:>4}", 0, 10)
    display.text(f"{line[4]:>4}", 64, 10)
    display.show()

    # === Edge Sensor Values ===
    left = line[0]
    right = line[4]

    # Strong & confident detection condition
    if (left > border_threshold or right > border_threshold) and max(left, right) > strong_black_threshold:
        confirm_counter += 1
    else:
        confirm_counter = 0

    # === Trigger Turn ===
    if confirm_counter >= confirm_required:
        motors.set_speeds(3000, -3000)
        time.sleep_ms(turn_duration_ms)
        motors.set_speeds(0, 0)
        time.sleep_ms(100)

        motors.set_speeds(drive_speed, drive_speed)
        time.sleep_ms(move_forward_after_turn_ms)

        confirm_counter = 0

    else:
        motors.set_speeds(drive_speed, drive_speed)

    time.sleep_ms(80)
