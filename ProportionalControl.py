from sbot import *


# Helper to set both motors
def set_motors(left, right):
    motors.set_power(0, left)
    motors.set_power(1, right)

# Helper to find the target marker in the list of markers
def find_target(markerlist, target):
    for marker in markerlist:
        if marker.id == target:
            return marker
    return None

# Function to drive toward a marker until a specific distance
def drive_to_marker(target_id, target_distance):
    while True:
        markerlist = vision.detect_markers()
        target_marker = find_target(markerlist, target_id)
        if target_marker is not None:
            distance_error = target_marker.position.distance - target_distance
            angle_error = target_marker.position.horizontal_angle

            if distance_error > 0:  # Still too far from the marker
                if angle_error > 0.2:  # Target is on the right
                    set_motors(0.1, -0.1)
                elif angle_error < -0.2:  # Target is on the left
                    set_motors(-0.1, 0.1)
                else:  # Target is straight ahead
                    set_motors(0.5, 0.5)
            else:  # Reached the target distance
                set_motors(0, 0)
                break

def proportionalControl(Kp, IR, baseValue,expectedValue ):
    # Combine side error and center deviation to calculate the overall error
    error =  expectedValue- IR

    # Calculate motor speeds using proportional control
    base_speed = baseValue/max(0.001, abs(error))
    correction = Kp * error

    left_speed = base_speed - correction
    right_speed = base_speed + correction
    print(left_speed, right_speed)

    # Ensure motor speeds are within valid range (0 to 1)
    left_speed = max(-1, min(1, left_speed))
    right_speed = max(-1, min(1, right_speed))

    # Set motor speeds
    set_motors(left_speed, right_speed)


current_state = "forward"
while True:
    # Read IR sensor values
    left_IR = arduino.analog_read(AnalogPin.A0)
    centre_IR = arduino.analog_read(AnalogPin.A1)
    right_IR = arduino.analog_read(AnalogPin.A2)

    print(left_IR, centre_IR, right_IR)

    Kp = 0.4 # Proportional gain
    BaseValue = 0.2
    
    def set_state(state):
        print(current_state)
        if state == "forward":
            set_motors(0.25,0.25)
        elif state == "left":
            Kp= 0.05
            proportionalControl(Kp, left_IR, BaseValue,2)
        elif state == "left black":
            Kp = 0.1
            proportionalControl(Kp, left_IR, BaseValue*0.1,5)
        elif state == "right":
            Kp=-0.05
            proportionalControl(Kp, right_IR, BaseValue,2)
        elif state == "right black":
            Kp=-0.1
            proportionalControl(Kp, right_IR, BaseValue*0.1, 5)
        elif state == "vision":
            drive_to_marker(1, 1200)  # Drive to marker 1 until 700mm away
            set_motors(-0.05, 0.05)  # Rotate in place
            utils.sleep(0.015)
            drive_to_marker(3, 1000)  # Drive to marker 3 until 700mm away
            set_motors(0.05, -0.05)  # Rotate in place
            utils.sleep(0.015)
            drive_to_marker(5, 1000)  # Drive to marker 5 until 700mm away
            set_motors(-0.05, 0.05)  # Rotate in place
            utils.sleep(0.015)
            drive_to_marker(7, 1000)  # Drive to marker 7 until 700mm away
            set_motors(-0.05, 0.05)  # Rotate in place
            utils.sleep(0.015)
            drive_to_marker(1, 1000)  # Drive to marker 9 until 700mm away
            set_motors(-0.05, 0.05)  # Rotate in place
            utils.sleep(0.015)
            drive_to_marker(3, 1000)  # Drive to marker 3 until 700mm away
            set_motors(0.05, -0.05)  # Rotate in place
            utils.sleep(0.015)
            drive_to_marker(5, 1000)  # Drive to marker 5 until 700mm away
            set_motors(-0.05, 0.05)  # Rotate in place
            utils.sleep(0.015)
            drive_to_marker(7, 1000)  # Drive to marker 7 until 700mm away
            set_motors(-.05, 0.05)  # Rotate in place
            utils.sleep(0.015)


    if left_IR < 1.2 and current_state!= "left black":
        current_state = "left"
        timer = 0
    elif left_IR >3.5:
        current_state = "left black"
        timer = 0
    elif right_IR < 1.2 and current_state != "right black":
        current_state = "right"
        timer = 0
    elif right_IR >3.5:
        current_state = "right black"
        timer = 0
    elif left_IR > 1.2 and left_IR < 3.5 and right_IR > 1.2 and right_IR < 3.5:
        timer += 1
    elif centre_IR > 3.5:
        current_state = "forward"
        
        timer = 0
    if timer > 10:
        current_state = "vision"
    set_state(current_state)

while False:
    Kp = 0.1 # Proportional gain

    # Combine side error and center deviation to calculate the overall error
    error = 3 - right_IR

    # Calculate motor speeds using proportional control
    base_speed = 0.2/max(0.001, abs(error))
    correction = Kp * error

    left_speed = base_speed - correction
    right_speed = base_speed + correction
    print(left_speed, right_speed)

    # Ensure motor speeds are within valid range (0 to 1)
    left_speed = max(-1, min(1, left_speed))
    right_speed = max(-1, min(1, right_speed))

    # Set motor speeds
    set_motors(left_speed, right_speed)
