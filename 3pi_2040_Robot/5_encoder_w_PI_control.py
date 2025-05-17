"""
This implements the proportional part of a PID controller with some display data

Pseudocode:

Initialize system
Set Kp (proportional gain)
Set target_speed

Loop forever:
    current_time = get current time
    dt = current_time - last_time

    IF enough time has passed (e.g. 100 ms):
        last_time = current_time

        Read encoder ticks since last time
        Reset encoder

        Calculate current_speed = ticks / dt

        error = target_speed - current_speed
        control_output = Kp * error

        pwm = convert control_output to valid PWM range (0–max)

        Set motor speed using pwm

        Optionally: display current_speed and pwm

"""
from pololu_3pi_2040_robot import robot
import time

encoders = robot.Encoders()
button_a = robot.ButtonA()
motors = robot.Motors()
display = robot.Display()

target_value = 6000  # How far we will travel
speed = motors.MAX_SPEED // 2
"""
To set the kp we will use:
    Kp = desired_correction / error
    For example if we want to compensate by a motor speed of 1000 when there is a 50 tick imbalance (error), we set kp to ....
    Kp = 1000 / 50 = 20
    This can be tuned for better performance:

        If robot drifts: increase Kp to 25–30
        If robot oscillates or wiggles: reduce Kp to 10–15
"""
kp = 30
ki = 1

last_time = time.ticks_ms()
run = False
"""
We can include a 'clamp' to prevent overcorrection. For example, we will keep speed to -2000 < s < 2000
"""
# correction = kp * error
# correction = max((-2000, min(2000, kp * error)))
total_error = 0

while True:
    # Track change in time
    current_time = time.ticks_ms()
    dt = time.ticks_diff(current_time, last_time) /1000 # Convert ms to seconds
    last_time = current_time


    # Read encoders
    c = encoders.get_counts()
    left_count = c[0]
    right_count = c[1]


    # Proportional/Integral control
    error = left_count - right_count
    correction = (kp * error) + (ki * total_error) # P + I terms

    # Calculate speed of each side: Slow down whichever side is ahead, speed up whichever side is behind
    left_pwm = speed - correction
    right_pwm = speed + correction

    # Clamp the speed values in case the error gets overly large to ensure it stays within the allowable motor range
    left_pwm = max(-speed, min(speed, left_pwm))
    right_pwm = max(-speed, min(speed, right_pwm))

    #Show info on the screen
    display.fill_rect(0, 0, 128, 30, 0)
    display.text("Left: " + str(c[0]), 0, 0)
    display.text("Right: " + str(c[1]), 0, 10)
    display.text("Error:" + str(total_error), 0, 20)


    if button_a.check() and button_a.is_pressed(): # Run the Robot and set initial conditions
        if not run:
            time.sleep(0.5)
            run = True
            encoders.get_counts(reset=True)
            total_error = 0 # Reset the I term
            last_time = time.ticks_ms() # Reset time

        else:
            run = False

    # Motor Control
    if run == True:
        motors.set_speeds(left_pwm, right_pwm)  # Drive the motors
        total_error += error * dt  # Accumulated error scaled up by dt (change in time)
        total_error = max(-1000, min(1000, total_error)) # Prevent error from accumulating too much

    if run == False or left_count >= target_value or right_count >= target_value:
        run = False
        motors.set_speeds(0, 0)  # Drive the motors


    time.sleep_ms(50)
    display.show()

