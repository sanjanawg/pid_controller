from machine import Pin, I2C, PWM 
import time
from pid import PID  # Import the PID library

# Constants for the SHT40 sensor
SHT40_ADDRESS = 0x44  # I2C address for SHT40
MEASURE_HIGH_PRECISION = b'\xFD'  # High precision measurement command

# Initialize I2C (GPIO pins can be changed based on your setup)
i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=100000)

# Initialize PWM on GPIO Pin 21
pwm_pin = PWM(Pin(21))
pwm_pin.freq(1000)  # Set PWM frequency to 1 kHz

# Setup PID controller
setpoint = 37.0  # Desired temperature in Celsius
pid = PID(4, 4.5, 0.07, setpoint)

# Define anti-windup limits for the integral term
INTEGRAL_MIN = -500  
INTEGRAL_MAX = 500  

# Function to read temperature from SHT40
def read_sht40():
    try:
        # Send the measurement command to the SHT40
        i2c.writeto(SHT40_ADDRESS, MEASURE_HIGH_PRECISION)
        time.sleep_ms(10)  # Wait for the measurement to complete

        # Read 6 bytes of data: 2 bytes temperature, 1 byte CRC, 2 bytes humidity, 1 byte CRC
        data = i2c.readfrom(SHT40_ADDRESS, 6)

        # Parse temperature raw values
        temperature_raw = (data[0] << 8) | data[1]

        # Convert raw values to temperature (°C)
        temperature = -45 + (175 * (temperature_raw / 65535.0))

        return temperature

    except Exception as e:
        print("Error reading SHT40:", e)
        return None

# PID control function with anti-windup
def pid_control(pid, temperature):
    error = pid.setpoint - temperature

    # Initialize last_error if None
    if pid._last_error is None:
        pid._last_error = 0.0  # Ensures no NoneType error

    # Compute proportional term
    proportional = pid.Kp * error

    # Compute and limit integral term (Anti-windup)
    pid._integral += pid.Ki * error
    pid._integral = max(INTEGRAL_MIN, min(INTEGRAL_MAX, pid._integral))  # Clamp integral

    # Compute derivative term safely
    derivative = pid.Kd * (error - pid._last_error)

    # Compute full control signal
    control_signal = proportional + pid._integral + derivative

    # Prevent integral windup when output is saturated
    if control_signal > 65535:
        pid._integral -= (control_signal - 65535) * 0.1  # Reduce integral accumulation
    elif control_signal < 0:
        pid._integral -= (control_signal - 0) * 0.1  # Reduce integral accumulation

    # Update last error
    pid._last_error = error

    return control_signal

# Main loop for temperature control
while True:
    temperature = read_sht40()
    if temperature is not None:
        print(f"Temperature: {temperature:.2f} °C")

        # Get the PID control signal using anti-windup function
        control_signal = pid_control(pid, temperature)

        # Map the control signal to a PWM duty cycle (ensure proper scaling)
        duty_cycle = max(0, min(65535, int((control_signal / 100) * 65535)))  # Adjusted Scaling

        # Apply PWM to the Peltier module
        pwm_pin.duty_u16(duty_cycle)

    else:
        print("Failed to read temperature. Skipping control.")

    time.sleep(1)  # Delay for 1 second before next reading

