from machine import Pin, PWM, ADC
import utime

# Define servo PWM pins
servo_x = PWM(Pin(15))
servo_y = PWM(Pin(16))
servo_x.freq(50)
servo_y.freq(50)

# Define joystick ADC pins
joystick_x = ADC(Pin(26))
joystick_y = ADC(Pin(27))

# PID parameters
Kp = 0.5  # Proportional gain
Ki = 0.01  # Integral gain
Kd = 0.1  # Derivative gain

# PID error tracking
prev_error_x = 0
prev_error_y = 0
integral_x = 0
integral_y = 0

# Function to map joystick values to servo PWM
def map_value(value, in_min, in_max, out_min, out_max):
    return int(out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min))

while True:
    # Read joystick values
    x_val = joystick_x.read_u16()
    y_val = joystick_y.read_u16()
    
    # Calculate error (desired - actual)
    target_x = map_value(x_val, 0, 65535, 2000, 8000)
    target_y = map_value(y_val, 0, 65535, 2000, 8000)
    
    error_x = target_x - prev_error_x
    error_y = target_y - prev_error_y
    
    # Calculate integral
    integral_x += error_x
    integral_y += error_y

    # Calculate derivative
    derivative_x = error_x - prev_error_x
    derivative_y = error_y - prev_error_y

    # PID control output
    output_x = int((Kp * error_x) + (Ki * integral_x) + (Kd * derivative_x))
    output_y = int((Kp * error_y) + (Ki * integral_y) + (Kd * derivative_y))

    # Update servo position
    servo_x.duty_u16(max(min(target_x + output_x, 8000), 2000))  # Constrain output
    servo_y.duty_u16(max(min(target_y + output_y, 8000), 2000))

    # Store previous error
    prev_error_x = error_x
    prev_error_y = error_y
    
    utime.sleep(0.05)  # Small delay for stability