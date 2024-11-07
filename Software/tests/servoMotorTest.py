from machine import Pin, PWM
import time

# Function to convert angle to PWM duty
def angle_to_duty(angle):
    min_duty = 26
    max_duty = 128
    return int(min_duty + (angle / 180.0) * (max_duty - min_duty))

# Setup PWM and servo
pwm_pin = Pin(17) # Change with pin configuration
servo = PWM(pwm_pin, freq=50)

# Function to set servo to a specific angle
def set_servo_angle(angle):
    duty = angle_to_duty(angle)
    servo.duty(duty)

startTime = time.time()
maxTime = 30 #seconds
currentTime = time.time()
angle = 0
while currentTime - startTime <= maxTime:
    try:
        angle += 10
        angle = angle%360
        set_servo_angle(angle)
        time.sleep(0.01)
    except KeyboardInterrupt:
        # Use Ctrl-C to end the test early
        print("Test ended.")
        break