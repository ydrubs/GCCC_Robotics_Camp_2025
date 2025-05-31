"""
Wind Sprints Challenge 🏁

Overview:
This program helps you complete the Wind Sprints robotics challenge using the 3pi+ robot.
The goal is to drive forward and cross black lines made with electrical tape, then return
to the starting line after each run — just like running wind sprints in sports.

Challenge Rules:
- You will create 3 black lines spaced out on a white surface using electrical tape.
- The robot starts behind the first (starting) line.
- It must cross the lines one at a time, then always return to the starting line before continuing.
  1. Go to the first line and return — earn 10 points.
  2. Go to the second line and return — earn 10 more points.
  3. Go to the third line and return — earn 10 more points.
- After finishing, the robot must stop after crossing the starting line to earn 5 bonus points.

Scoring Breakdown:
✅ 10 points for clearing the first line
✅ 10 points for clearing the second line
✅ 10 points for clearing the third line
✅  5 bonus points for stopping correctly after returning to the starting line

🎯 Maximum score: 35 points

Robot Behavior:
- The robot plays a short buzzer sound when it crosses a line.
- It displays the number of forward and backward line crossings on its screen.
- Your task is to program the robot to use this information to make movement decisions.

Calibration Instructions:
1. Place the robot on a surface with black and white areas.
2. When prompted, press **Button A** to begin sensor calibration.
   The robot will spin to sample both black and white for better accuracy.
3. Press **Button A** again to start the challenge.

Now it’s your turn to build the logic that lets your robot sprint, turn, return, and stop!
👉 You can lay out your decision-making logic in the **line counting section** of the script.
👉 The green text offers suggestions to code the solution, but it is not the only way to approach this
👉 Movement commands are handled in the **movement section**
👉 You can also add **custom moves** like spins, pauses, or speed changes to help solve the challenge in your own way.
"""

from pololu_3pi_2040_robot import robot
import time
from _9_line_cross_controller import LineCrossController

# === Hardware ===
motors = robot.Motors()
display = robot.Display()

# === Setup ===
controller = LineCrossController() # Handles the logic of checking if a line is crossed
controller.calibrate() # Calibrates the line sensors

# === Variables ===
forward_count = 0 # counts lines crossed in the forward direction
backward_count = 0 # counts lines crossed in the backward direction
direction = "forward" # Sets the direction
stage = 1 # Sets what line we are currently working on

# === Main Loop ===
while True:
    crossed = controller.crossed_line() # Checks if a line is crossed

    # --- Line Counting ---
    if crossed: # If a line is crossed, beep and execute logic below -

        # === STAGE 1 ===
        if stage == 1:  # Trying to cross 1 black line and return
            if direction == "forward":
                # === STUDENT TASK ===
                """
                ✅ GOAL: Cross the first black line (after the starting line)

                ❓ QUESTIONS:
                - How many lines do we need to cross?
                - What should happen to the  `forward_count` variable?
                - When should we switch to 'backward' direction?
                - How do we handle counting lines in the backward direction?
                - when do we know that we have crossed back over the starting line after moving forward? 

                💡 HINT: We need to reset both forward_count and backward_count before going to stage 2 
                   IMPORTANT: We need to use the 'continue' keyword to force the while loop to restart on stage 2 
                """


            elif direction == "backward":
                # === STUDENT TASK ===
                """
                ✅ GOAL: Return back over the first black line and the starting line.
                         The logic will be very similar to stage 1

                ❓ QUESTIONS:
                - What should happen to `backward_count`?
                - When should we move to stage 2?

                💡 HINT: Use the `continue` keyword to avoid running stage 2 logic early
                """



        # === STAGE 2 ===
        elif stage == 2:  # Trying to cross 2 lines and return
            if direction == "forward":
                """
                ✅ GOAL: Cross two black lines (after the starting line)

                ❓ QUESTIONS:
                - What should happen to `forward_count`?
                - When do we switch to 'backward'?

                💡 HINT: Increment and compare
                """


            elif direction == "backward":
                """
                ✅ GOAL: Return back over both black lines and the starting line

                ❓ QUESTIONS:
                - What should happen to `backward_count`?
                - When should we move to stage 3?

                💡 HINT: Reset counters again and continue
                """



        # === STAGE 3 ===
        elif stage == 3:  # Trying to cross 3 lines and return
            if direction == "forward":
                """
                ✅ GOAL: Cross three black lines (after the starting line)

                ❓ QUESTIONS:
                - What should happen to `forward_count`?
                - When do we switch to 'backward'?
                """


            elif direction == "backward":
                """
                ✅ GOAL: Return back over all three lines and stop

                ❓ QUESTIONS:
                - What should happen to `backward_count`?
                - When are we completely done?

                💡 HINT: You may not need to reset anymore. Try stopping the robot bt using direction = 'done'
                """


    # --- Movement ---
    if direction == "forward":
        motors.set_speeds(1500, 1500)
    elif direction == "backward":
        motors.set_speeds(-1500, -1500)
    elif direction == "turn":
        motors.set_speeds(-1500, 1500)
        time.sleep(3)
    else:
        motors.off()

    # --- Display Feedback ---
    display.fill(0)
    display.text(f"Fwd: {forward_count}", 0, 0)
    display.text(f"Bck: {backward_count}", 0, 10)
    display.show()

    time.sleep(0.01)
