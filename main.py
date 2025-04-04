from machine import Pin, PWM, ADC
import utime

# Define servo PWM pins
servo_x = PWM(Pin(0))  # Adjust as needed
servo_y = PWM(Pin(16))  # Adjust as needed
servo_x.freq(50)  # Standard servo frequency (50 Hz)
servo_y.freq(50)

# Define joystick ADC pins
joystick_x = ADC(Pin(26))  # Adjust to your wiring
joystick_y = ADC(Pin(27))

# Function to map joystick values to servo PWM
def map_value(value, in_min, in_max, out_min, out_max):
    return int(out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min))

while True:
    # Read joystick values
    x_val = joystick_x.read_u16()
    y_val = joystick_y.read_u16()
    
    # Map values to servo positions
    servo_x.duty_u16(map_value(x_val, 0, 65535, 2000, 8000))  # Adjust for your servo range
    servo_y.duty_u16(map_value(y_val, 0, 65535, 2000, 8000))
    
    utime.sleep(0.05)  # Small delay for stability