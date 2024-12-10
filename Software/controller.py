# Main control code
from machine import ADC,Pin
import time
from time import sleep_ms
from hcsr04 import HCSR04
from MPU6050 import MPU6050
from mg90s import servo
import pc

#----- Pinout declaration
# Joystick pins
pinVrx = 5
pinVry = 6
pinSW  = 7

# Ultrasonic pins
us1trig = 16
us1echo = 15
us2trig = 14
us2echo = 13

# Gyroscope pins
gyroSclPin = 37
gyroSdaPin = 38

# Servo pin
servoPin = 11

#----- Device config
# Joystick
xButton = ADC(Pin(pinVrx, Pin.IN)) # create an ADC object acting on a pin
xButton.atten(xButton.ATTN_11DB)
xButtZero = xButton.read()
yButton = ADC(Pin(pinVry, Pin.IN)) # create an ADC object acting on a pin
yButton.atten(yButton.ATTN_11DB)
yButtZero = yButton.read()
button = Pin(pinSW, Pin.IN, Pin.PULL_UP)

# Ultrasonics
sensor1 = HCSR04(trigger_pin=us1trig, echo_pin=us1echo)
sensor2 = HCSR04(trigger_pin=us2trig, echo_pin=us2echo)

# Gyroscope
mpu = MPU6050(sclPin = gyroSclPin, sdaPin = gyroSdaPin)

# Motor
motor = servo(servoPin)

#----- Auxilliary Functions
minAngle = 1.8
maxAngle = 2.65
def boundsOK():
    angle = mpu.read_angle()['y']
    if angle<minAngle:
        motor.set_speed(0.08)
        return False
    if angle>maxAngle:
        motor.set_speed(-0.08)
        return False
    return True

zero = 2.27 # Must be experimentally determined
tolerance = 0.04 # radians
def center():
    ready = input("Begin leveling sequence?")
    if not ready:
        print("Leveling cancelled")
        return None
    print("Centering...")
    diff = 999
    while abs(diff)>tolerance: 
        try:
            angle = mpu.read_angle()['y']
            diff = zero-angle
            print(f"Angle: {angle:.4g}; diff: {diff:.4g}")
            if boundsOK():
                sign = diff/abs(diff)
                motor.set_speed(sign*0.05)
                time.sleep(0.02)
            else:
                raise ValueError("Bounds exceeded during centering.")
                return None
        except KeyboardInterrupt:
            print("Leveling cancelled")
            break
    motor.set_speed(0)
    print("Initial leveling complete")
    return None

L = 410 # mm
def position():
    s1 = sensor1.distance_mm()
    s2 = sensor2.distance_mm()
    if s1<s2:
        return s1
    else:
        return L - s2
    
#----- Main loop
center()
mode = 'manual'
# PID configuration
Kc = 50
KI = 0
KD = 370
pid = pc.PID(Kc, KI, KD, dt=0.1, \
             outputLimits=[-1, 1])
while True:
    try:
        if mode == 'manual':
            yMaxRead = 700
            if boundsOK():
                stick = (yButtZero-yButton.read())/yMaxRead
                if abs(stick) < 0.05:
                    stick = 0
                reading = max(min(1, stick), -1)
                motor.set_speed(reading)
            if button.value() == 0:
                mode = 'auto'
        elif mode == 'auto':            
            setpoint = L/2
            pid.setpoint = setpoint
            if boundsOK():
                pos = position()
                print(f"Position = {pos:3g} mm")
                newSpeed = pid(pos)[0]
                motor.set_speed(newSpeed)
            if button.value() == 0:
                mode = 'manual'
        else:
            print("Unknown error...")
    except KeyboardInterrupt:
        print("Loop ended.")
        break
