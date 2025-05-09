from pololu_3pi_2040_robot import robot
import time


rgb_leds = robot.RGBLEDs()
rgb_leds.set_brightness(5)

colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255], [128, 0, 128], [255, 255, 0], [255, 255, 255]]
num_colors = len(colors)

while True: # Need this for the LED's to stay on

    for i in range(num_colors):
        # Cycle LED 0, 1, and 2 through the colors with an offset
        rgb_leds.set(0, colors[i % num_colors])
        rgb_leds.set(1, colors[(i + 1) % num_colors])
        rgb_leds.set(2, colors[(i + 2) % num_colors])
        rgb_leds.set(3, colors[(i + 3) % num_colors])
        rgb_leds.set(4, colors[(i + 4) % num_colors])
        rgb_leds.set(5, colors[(i + 5) % num_colors])

    # rgb_leds.set_hsv(0, [0, 255, 255])
        time.sleep(0.25)  # adjust timing as needed
        rgb_leds.show()