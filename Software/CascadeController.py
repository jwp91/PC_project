# Main control code
from machine import ADC,Pin
import time
from time import sleep_ms
from hcsr04 import HCSR04
from MPU6050 import MPU6050
from mg90s import servo
import pc
pi = 3.141592

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
yButtZero = 7436
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
        motor.set_speed(-0.1)
        return False
    if angle>maxAngle:
        motor.set_speed(0.1)
        return False
    return True

zero = 2.29 # Must be experimentally determined
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
            diff = angle-zero
            print(f"Angle: {angle:.4g}; diff: {diff:.4g}")
            if boundsOK():
                sign = diff/abs(diff)
                motor.set_speed(sign*0.1)
                time.sleep(0.02)
            else:
                motor.set_speed(0)
                raise ValueError("Bounds exceeded during centering.")
                return None
        except KeyboardInterrupt:
            print("Leveling cancelled")
            break
    motor.set_speed(0)
    print("Initial leveling complete")
    return None

L = 405 # mm
posLast = L/2
rad = 20 #mm
def position(alpha = 0.1, output = False):
    global posLast
    s1 = sensor1.distance_mm() + rad
    time.sleep(0.005) # Time for a ping to return at max distance
    s2 = sensor2.distance_mm() + rad
    
    # Gate filter
    if s1 < 0:
        s1 = 0
    if s2 < 0:
        s2 = 0
    if s1 > L:
        s1 = L
    if s2 > L:
        s2 = L
    if s1<s2:
        reading = s1
    else:
        reading = L - s2
    if abs(reading-posLast) > 150:
        reading = posLast
    posLast = alpha*reading + (1-alpha)*posLast
    if output:
        print(f"s1: {s1:.4g}, s2: {L-s2:.4g}, SP: {L/2}")
    return posLast
    
#----- Main loop
center()
mode = 'manual'
# Outer PID configuration (position error -> angle setpoint)
Kc_out = pi/250
KI_out = 0
KD_out = 2*Kc_out
# Optimization showed that KD_out ~= 2*Kc_out
pidOuter = pc.PID(Kc_out, KI_out, KD_out, dt=0.1, \
             outputLimits=[minAngle, maxAngle])
# Inner PID configuration (angle error -> motor speed)
Kc_in = -6.4 # Simulation used a range of -25 to 25 instead of -1 to 1
KI_in = 0.0
KD_in = 0.0
pidInner = pc.PID(Kc_in, KI_in, KD_in, dt = 0.1, \
                  outputLimits=[-1, 1])
startTime = time.time() # seconds
reading = 0
print("Mode: manual")
try:
    while True:
        if mode == 'manual':
            stick = yButton.read()-yButtZero
            if abs(stick) < 100:
                stick = 0
            reading = max(min(1, stick), -1)
            if boundsOK():
                motor.set_speed(reading)
            else:
                print("Bounds error! Correcting...")
            pos = position(output = False)
            thetaActual = mpu.read_angle()['y']
            newSpeed = reading
            print(f"pos: {pos:.4g}, posSP: {L/2:.4g}")
            if button.value() == 0 and (time.time()-startTime)%0.5<0.01:
                # The extra bit in the "and" statement reduces the
                # button's sensitivity by effectively only having it
                # read every half second.
                mode = 'auto'
                print("Mode: auto")
        elif mode == 'auto':
            # Lengths should be meters to match PID params
            SP = L/2
            pos = position()
            pidOuter.setpoint = SP
            thetaSP, Pout, Iout, Dout = pidOuter(pos)
            pidInner.setpoint = thetaSP
            thetaActual = mpu.read_angle()['y']
            newSpeed = pidInner(thetaActual)[0]
            if boundsOK():
                motor.set_speed(newSpeed)
            else:
                pass
                #print("Bounds error! Correcting...")
            #print(f"pos: {pos:.4g}, posSP: {SP:.4g}")
            print(f"thetaSP: {thetaSP:.4g}, theta: {thetaActual:.4g}")
            if button.value() == 0 and (time.time()-startTime)%0.5<0.01:
                mode = 'manual'
                print("Mode: manual")
        else:
            print("Unknown error...")
except KeyboardInterrupt:
    print("Loop ended.")
    motor.set_speed(0)
