from sbot import *

while False:
    # Read IR sensor values
    left_IR = arduino.analog_read(AnalogPin.A0)
    centre_IR = arduino.analog_read(AnalogPin.A1)
    right_IR = arduino.analog_read(AnalogPin.A2)

    print(left_IR, centre_IR, right_IR)
    def set_state(state):
        if state == "forward":
            set_motors(0.25,0.25)
        elif state == "left":
            set_motors(-0.1,0.3)
        elif state == "right":
            set_motors(0.2,-0.1)

    if left_IR < 1.2 and right_IR < 1.2:
        current_state = "forward"
    else:
        if left_IR > 3.5 or left_IR < 1.2:
            current_state = "left"
        if left_IR > 1.2 and left_IR < 3.5:
            current_state = "right"
    set_state(current_state)
        
while False:
    # Helper function to clamp values within a range
    def clamp(value, min_val, max_val):
        return max(min_val, min(max_val, value))

    # Read IR sensor values
    left_IR = arduino.analog_read(AnalogPin.A0)
    centre_IR = arduino.analog_read(AnalogPin.A1)
    right_IR = arduino.analog_read(AnalogPin.A2)

    print(centre_IR)

    error = 2 - centre_IR
    leftSpeed = 0.2 + (2 - right_IR) * 0.1
    rightSpeed = 0.2 - (2 - left_IR) * 0.1

    # Ensure motor speeds are within valid range (e.g., 0 to 1)
    leftSpeed = clamp(leftSpeed, 0, 1)
    rightSpeed = clamp(rightSpeed, 0, 1)

    motors.set_power(0, leftSpeed)
    motors.set_power(1, rightSpeed)
    # set_motors function is already defined at the top level
    Kp = 0.125  # Proportional gain

    def set_motors(left, right):
        motors.set_power(0, left)
        motors.set_power(1, right)

    # Read IR sensor values
    left_IR = arduino.analog_read(AnalogPin.A0)
    centre_IR = arduino.analog_read(AnalogPin.A1)
    right_IR = arduino.analog_read(AnalogPin.A2)

    print(left_IR, centre_IR, right_IR)

    # Calculate error (assuming the target is to keep the robot centered)
    error = (right_IR - left_IR)

    # Calculate motor speeds using proportional control
    base_speed = 0.2  # Base speed for both motors
    correction = Kp * error

    left_speed = base_speed + correction
    right_speed = base_speed - correction

    # Ensure motor speeds are within valid range (e.g., 0 to 1)
    left_speed = clamp(left_speed, 0, 1)
    right_speed = clamp(right_speed, 0, 1)

    # Set motor speeds
    set_motors(left_speed, right_speed)

#while True:
    Kp = 0.05  # Proportional gain

    def set_motors(left, right):
        motors.set_power(0, left)
        motors.set_power(1, right)

    # Read IR sensor values
    left_IR = arduino.analog_read(AnalogPin.A0)
    centre_IR = arduino.analog_read(AnalogPin.A1)
    right_IR = arduino.analog_read(AnalogPin.A2)

    print(left_IR, centre_IR, right_IR)

    # Calculate error using all three sensors
    # The center IR sensor is weighted more heavily to prioritize staying on the line
    # Calculate the difference between left and right IR sensor values
    side_error = left_IR - right_IR

    # Calculate the deviation of the center IR sensor from the target value (2)
    center_deviation = (centre_IR - 2) * 0.5  # Weighted adjustment for center sensor

    # Combine side error and center deviation to calculate the overall error
    error = side_error + center_deviation

    # Calculate motor speeds using proportional control
    base_speed = 0.2  # Base speed for both motors
    correction = Kp * error

    left_speed = base_speed - correction
    right_speed = base_speed + correction

    # Ensure motor speeds are within valid range (0 to 1)
    left_speed = max(0, min(1, left_speed))
    right_speed = max(0, min(1, right_speed))

    # Set motor speeds
    set_motors(left_speed, right_speed)

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
                    set_motors(0.2, 0.2)
            else:  # Reached the target distance
                set_motors(0, 0)
                break
        else:  # Can't see the target, rotate to search
            set_motors(0.05, -0.05)  # Rotate in place
            utils.sleep(0.01)
        utils.sleep(0.2)

# Task 6: Drive toward markers in sequence
drive_to_marker(1, 500)  # Drive to marker 1 until 500mm away
drive_to_marker(3, 500)  # Drive to marker 3 until 300mm away
drive_to_marker(5, 500)  # Drive to marker 5 until 500mm away


from sbot import motors,utils,arduino,AnalogPin,BRAKE,COAST,PowerOutputPosition,vision


while True:
    left_IR = arduino.analog_read(AnalogPin.A0)
    centre_IR = arduino.analog_read(AnalogPin.A1)
    right_IR = arduino.analog_read(AnalogPin.A2)

    def set_motors(left, right):
        motors.set_power(0, left)
        motors.set_power(1, right)

    
    def savetoMove():
        distance = arduino.measure_ultrasound_distance(2, 3)
        if (distance <= 50) and (distance > 0):
            
            print("Ultrasound detected an obstacle")
            print(f"Distance: {distance} mm")
            current_state = "backward"
            set_state(current_state)
            utils.sleep(3)
            current_state = "left"
            set_state(current_state)
            utils.sleep(1)
            return False
        else:
            return True
    def default(state):
        if left_IR>1.2 and left_IR < 3.5:
            return "forward"


    def set_state(state):
        if state == "forward":
            set_motors(0.275,0.275)
        elif state == "left":
            set_motors(-0.2,0.2)
        elif state == "right":
            set_motors(0.2,-0.2)
        elif state == "stop":
            set_motors(0, 0)
        elif state == "backward":
            set_motors(-0.275, -0.275)
        elif state == "crazy":
            set_motors(1, 1)
            

    if (left_IR > 1.2 and left_IR < 3.5) and (right_IR > 1.2 and right_IR < 3.5):
        current_state = "forward"
        set_state(current_state)
        if savetoMove() == True:
            utils.sleep(0.5)
        
            

    if left_IR < 1.2 and right_IR < 1.2:
        current_state = "forward"
    else:
        if left_IR > 3.5 or left_IR < 1.2:
            current_state = "left"
        if left_IR > 1.2 and left_IR < 3.5:
            current_state = "right"
    set_state(current_state)




        

while False:
    Kp = 0.1 # Proportional gain

    def set_motors(left, right):
        motors.set_power(0, left)
        motors.set_power(1, right)

    # Read IR sensor values
    left_IR = arduino.analog_read(AnalogPin.A0)
    centre_IR = arduino.analog_read(AnalogPin.A1)
    right_IR = arduino.analog_read(AnalogPin.A2)

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
