"""
This program uses PID control (Proportional + Integral + Derivative)
to help the robot drive in a straight line by adjusting left and right motor speeds.

Students can tune:
    - Kp (Proportional): responds to current error
    - Ki (Integral): responds to accumulated past error
    - Kd (Derivative): responds to how quickly the error is changing

Try changing the values below and observe how the robot behaves.
"""

from pololu_3pi_2040_robot import robot
import time

# --- Hardware setup ---
encoders = robot.Encoders()
button_a = robot.ButtonA()
motors = robot.Motors()
display = robot.Display()

# === START STUDENT TUNABLE PARAMETERS ===

# Distance the robot should travel (6000 encoder ticks ≈ short straight run)
target_value = 8000

# Base speed of the motors (try 2000 to 3000 for testing)
speed = motors.MAX_SPEED // 2  # or set explicitly like: speed = 3000

# PID gains: try changing one at a time and observe
Kp = 30  # Proportional gain: corrects current error
Ki = 1   # Integral gain: corrects long-term drift
Kd = 0   # Derivative gain: smooths the response

# === END OF STUDENT TUNABLE PARAMETERS ===


# --- Control state ---
run = False
last_time = time.ticks_ms()
last_error = 0
total_error = 0

while True:
    # --- Time tracking for dt ---
    current_time = time.ticks_ms()
    dt = time.ticks_diff(current_time, last_time) / 1000  # Convert ms to seconds
    last_time = current_time

    # --- Read encoders (do NOT reset here) ---
    c = encoders.get_counts()
    left_count = c[0]
    right_count = c[1]

    # --- Compute PID terms ---
    error = left_count - right_count
    total_error += error * dt
    d_error = (error - last_error) / dt if dt > 0 else 0
    last_error = error

    # --- Clamp integral term to prevent runaway growth (windup) ---
    total_error = max(-1000, min(1000, total_error))

    # --- PID formula ---
    correction = (Kp * error) + (Ki * total_error) + (Kd * d_error)

    # --- Compute motor speeds ---
    left_pwm = speed - correction
    right_pwm = speed + correction

    # --- Clamp speeds to safe values ---
    left_pwm = max(-speed, min(speed, left_pwm))
    right_pwm = max(-speed, min(speed, right_pwm))

    # --- Display encoder values and PID error ---
    display.fill_rect(0, 0, 128, 40, 0)
    display.text("L:" + str(left_count), 0, 0)
    display.text("R:" + str(right_count), 64, 0)
    display.text("Err:" + str(int(error)), 0, 10)
    display.text("Int:" + str(int(total_error)), 0, 20)
    display.text("Der:" + str(int(d_error)), 64, 20)
    display.show()

    # --- Start/Stop logic using Button A ---
    if button_a.check() and button_a.is_pressed():
        if not run:
            time.sleep(0.5)  # Debounce
            run = True
            encoders.get_counts(reset=True)
            total_error = 0
            last_error = 0
            last_time = time.ticks_ms()
        else:
            run = False

    # --- Stop if distance reached or manually stopped ---
    if run and (left_count >= target_value or right_count >= target_value):
        run = False
        motors.set_speeds(0, 0)
        display.fill_rect(0, 0, 128, 30, 0)
        display.text("Target reached!", 0, 0)
        display.show()
        continue

    # --- Drive motors ---
    if run:
        motors.set_speeds(left_pwm, right_pwm)
    else:
        motors.set_speeds(0, 0)

    time.sleep_ms(50)
