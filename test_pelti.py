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
pid = PID(0.5, 2, 0.3, setpoint)  # You may need to tune these constants

def read_sht40():
    try:
        # Send the measurement command to the SHT40
        i2c.writeto(SHT40_ADDRESS, MEASURE_HIGH_PRECISION)
        time.sleep_ms(10)  # Wait for the measurement to complete

        # Read 6 bytes of data: 2 bytes temperature, 1 byte CRC, 2 bytes humidity, 1 byte CRC
        data = i2c.readfrom(SHT40_ADDRESS, 6)

        # Parse temperature and humidity raw values
        temperature_raw = (data[0] << 8) | data[1]
#         humidity_raw = (data[3] << 8) | data[4]

        # Convert raw values to temperature (°C) and humidity (%)
        temperature = -45 + (175 * (temperature_raw / 65535.0))
#         humidity = 100 * (humidity_raw / 65535.0)

        return temperature

    except Exception as e:
        print("Error reading SHT40:", e)
        return None

# Main loop to read temperature, humidity, and control PWM using PID
while True:
    temperature = read_sht40()
    if temperature is not None:
        print(f"Temperature: {temperature:.2f} °C")

        # Get the PID control signal based on the current temperature
        control_signal = pid(temperature)  # This gives the control signal

        # Map the control signal to a PWM duty cycle (adjust scaling if needed)
        duty_cycle = max(0, min(65535, int(control_signal * 1000)))  # Scale to PWM duty range

        # Apply PWM to the Peltier module based on the control signal
        pwm_pin.duty_u16(duty_cycle)

#         print(f"Control Signal: {control_signal:.2f}, Duty Cycle: {duty_cycle}")

    else:
        print("Failed to read temperature. Skipping control.")

    
    time.sleep(1)  # Delay for 1 second before next reading
