from sbot import motors, utils
from sbot import leds, Colour
from sbot import vision

# Initialize the motors and LEDs
motors.set_power(0, 0)  # Stop left motor
motors.set_power(1, 0)  # Stop right motor
leds.set_colour(0, Colour.RED)  # Set LED A to red
leds.set_colour(1, Colour.CYAN)  # Set LED B to cyan
leds.set_colour(2, Colour.OFF)  # Turn LED C off

try:
    while True:
        # Detect markers
        markers = vision.detect_markers(save="snapshot.jpg")

        if markers:
            marker_ids = [marker.id for marker in markers]

            if 2 in marker_ids:
                # Action for marker 2: Move forward
                leds.set_colour(2, Colour.GREEN)  # Indicate moving forward
                motors.set_power(0, 0.3)  # Left motor forward
                motors.set_power(1, 0.3)  # Right motor forward
                utils.sleep(2)  # Move for 2 seconds

            elif 3 in marker_ids:
                # Action for marker 3: Turn left
                leds.set_colour(2, Colour.YELLOW)  # Indicate turning
                motors.set_power(0, 0.1)  # Slow down left motor
                motors.set_power(1, 0.3)  # Keep right motor fast
                utils.sleep(1.5)  # Turn for 1.5 seconds

        else:
            # No markers detected: Stop
            leds.set_colour(2, Colour.RED)  # Indicate stopping
            motors.set_power(0, 0)  # Stop left motor
            motors.set_power(1, 0)  # Stop right motor

        utils.sleep(0.5)  # Small delay before the next detection

except KeyboardInterrupt:
    # Graceful shutdown on Ctrl+C
    motors.set_power(0, 0)
    motors.set_power(1, 0)
    leds.set_colour(2, Colour.OFF)
