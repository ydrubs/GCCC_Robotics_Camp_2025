import time
import random
from pololu_3pi_2040_robot import robot


rgb_leds = robot.RGBLEDs()
rgb_leds.set_brightness(5)
yellow_led = robot.YellowLED()

while True:

    """
    Sets the color of the 6 RGB LEDS
        syntax (led # [R-value, G-value, B-value])
        each channel is 1 byte (0-255)
    """
    rgb_leds.set(0, [255, 0, 0])
    rgb_leds.set(1, [0, 255, 0])
    rgb_leds.set(2, [0, 0, 255])
    rgb_leds.set(3, [255, 0, 255])
    rgb_leds.set(4, [0, 255, 255])
    rgb_leds.set(5, [255, 255, 0])

    rgb_leds.show()
    time.sleep(1)

    rgb_leds.set(0, [0, 0, 0])
    rgb_leds.set(1, [0, 0, 0])
    rgb_leds.set(2, [0, 0, 0])
    rgb_leds.set(3, [0, 0, 0])
    rgb_leds.set(4, [0, 0, 0])
    rgb_leds.set(5, [0, 0, 0])

    rgb_leds.show()
    time.sleep(1)


    """ Uncomment below to make the LED's flash a random color """
    # for i in range(6):
    #     rgb_leds.set(i, [
    #         random.randint(0,255),
    #         random.randint(0, 255),
    #         random.randint(0, 255),
    #     ])
    #
    # rgb_leds.show()
    # yellow_led.on() # Some reason does not turn on
    # time.sleep(1)
    #
    # rgb_leds.off()
    # yellow_led.off()
    # time.sleep(1)

# noinspection PyUnreachableCode
"""
Task:
    - Create a custom blink sequence using different colors (Hex Values) and LED's 
"""