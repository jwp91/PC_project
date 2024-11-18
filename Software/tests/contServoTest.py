from machine import Pin, PWM
from time import sleep
from mg90s import servo

# Configure motor
motor = servo(17)

# Run the test
motor.test()