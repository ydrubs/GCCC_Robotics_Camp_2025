# robot_dance_base.py

from pololu_3pi_2040_robot import robot
import time

class DanceController:
    def __init__(self):
        # --- Hardware setup ---
        self.buzzer = robot.Buzzer()
        self.button_c = robot.ButtonC()
        self.motors = robot.Motors()
        self.set_motors = self.motors.set_speeds  # Alias for simpler use
        self.rgb_leds = robot.RGBLEDs()
        self.rgb_leds.set_brightness(5)

        # --- Configurable parameters ---
        self.colors = []     # Must be passed in via set_led_colors()
        self.song = ""       # Must be passed in via set_music()
        self.dance_routine = self.default_dance_routine

        # --- Internal state ---
        self.step_counter = 0

    # --- Configuration methods ---
    def set_music(self, song_str):
        """Set the music string to be played."""
        self.song = song_str

    def set_led_colors(self, color_list):
        """Set the LED blink color pattern."""
        self.colors = color_list

    def set_dance_routine(self, routine_function):
        """Assign the dance routine to execute when music plays."""
        self.dance_routine = routine_function

    # --- Internal animation ---
    def blink_leds(self):
        """Cycle through the LED pattern based on step counter."""
        color_count = len(self.colors)
        for i in range(6):
            color = self.colors[(self.step_counter + i) % color_count] if color_count > 0 else [0, 0, 0]
            self.rgb_leds.set(i, color)
        self.rgb_leds.show()

    def run_move(self, duration_ms):
        """Blink LEDs for the duration of a movement step."""
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
            self.blink_leds()
            time.sleep(0.1)
        self.step_counter += 1

    # --- Execution ---
    def run_once(self):
        """Play the music and execute the assigned dance routine."""
        if not self.song:
            raise ValueError("No music has been set. Use set_music().")

        self.buzzer.play_in_background(self.song)
        self.dance_routine(self)  # Must accept one argument: the controller
        self.stop_all()

    def stop_all(self):
        """Stop all motors, music, and LEDs."""
        self.buzzer.off()
        self.motors.off()
        for i in range(6):
            self.rgb_leds.set(i, [0, 0, 0])
        self.rgb_leds.show()

    def default_dance_routine(self, controller):
        """Fallback no-op routine (used if none provided)."""
        pass
