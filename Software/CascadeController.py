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
    sleep(0.0012) # Time for a ping at max distance
    s2 = sensor2.distance_mm()
    if s1<s2:
        return s1
    else:
        return L - s2
    
#----- Main loop
center()
mode = 'manual'
# Outer PID configuration (position error -> angle setpoint)
Kc_out = 250
KI_out = 0
KD_out = 450
pidOuter = pc.PID(Kc_out, KI_out, KD_out, dt=0.1, \
             outputLimits=[minAngle, maxAngle])
# Inner PID configuration (angle error -> motor speed)
Kc_in = 10.0/25 # Simulation used a range of -25 to 25 instead of -1 to 1
KI_in = 0.0
KD_in = 0.0
pidInner = pc.PID(Kc_in, KI_in, KD_in, dt = 0.1, \
                  outputLimits=[-1, 1])
startTime = time.time() # seconds
reading = 0
while True:
    try:
        if mode == 'manual':
            yMaxRead = 700
            stick = (yButtZero-yButton.read())/yMaxRead
            if abs(stick) < 0.05:
                stick = 0
            reading = max(min(1, stick), -1)
            if boundsOK():
                motor.set_speed(reading)
            else:
                print("Bounds error! Correcting...")
            pos = position()
            thetaActual = mpu.read_angle()['y']
            newSpeed = reading
            print(f"pos: {pos:.4g}, theta: {thetaActual:.4g}, speedSP: {newSpeed:.4g}")
            if button.value() == 0 and (time.time()-startTime)%0.5<0.01:
                # The extra bit in the "and" statement reduces the
                # button's sensitivity by effectively only having it
                # read every half second.
                mode = 'auto'
        elif mode == 'auto':            
            SP = L/2
            pos = position()
            pidOuter.setpoint = SP
            thetaSP = pidOuter(pos)[0]
            pidInner.setpoint = thetaSP
            thetaActual = mpu.read_angle()['y']
            newSpeed = pidInner(thetaActual)[0]
            if boundsOK():
                motor.set_speed(newSpeed)
            else:
                print("Bounds error! Correcting...")
            print(f"pos: {pos:.4g}, theta: {thetaActual:.4g}, speedSP: {newSpeed:.4g}")
            if button.value() == 0 and (time.time()-startTime)%0.5<0.01:
                mode = 'manual'
        else:
            print("Unknown error...")
    except KeyboardInterrupt:
        print("Loop ended.")
        break
