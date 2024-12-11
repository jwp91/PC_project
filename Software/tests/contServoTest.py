from machine import Pin, PWM
from time import sleep
from mg90s import servo

# Configure motor
motor = servo(11)

# Run the test
#motor.test()
motor.set_speed(0)
sleep(1)
speed = 0.5
motor.set_speed(speed)
print(speed)
sleep(1)
# motor.set_speed(-0.4)
# print(0.4)
# sleep(1)
# motor.set_speed(-0.6)
# print(0.6)
# sleep(1)
# motor.set_speed(-0.8)
# print(0.8)
# sleep(1)
# motor.set_speed(-1.0)
# print(1.0)
# sleep(1)
motor.set_speed(0.0)