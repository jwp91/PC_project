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
yButtonMax = 8191
yButtonMin = 0
yButtonMid = yButton.read()
button = Pin(pinSW, Pin.IN, Pin.PULL_UP)

# Ultrasonics
sensor1 = HCSR04(trigger_pin=us1trig, echo_pin=us1echo)
sensor2 = HCSR04(trigger_pin=us2trig, echo_pin=us2echo)

# Gyroscope
mpu = MPU6050(sclPin = gyroSclPin, sdaPin = gyroSdaPin)

# Motor
motor = servo(servoPin)

#----- Auxilliary Functions
#minAngle = 2.0  # 1.8 absolute
#maxAngle = 2.54 # 2.74 absolute
minAngle = 1.8
maxAngle = 2.74

midAngle = (maxAngle-minAngle)/2+minAngle
def boundsOK():
    angle = mpu.read_angle()['y']
    if angle<minAngle:
        motor.set_speed(-0.2)
        return False
    if angle>maxAngle:
        motor.set_speed(0.2)
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
def position(output = False):
    global posLast
    alpha = 0.1
    s1 = sensor1.distance_mm() + rad
    time.sleep(0.005) # Time for a ping to return at max distance
    s2 = sensor2.distance_mm() + rad
    
    # Gate filter
    if s1 < 50:
        s1 = 0
    if s2 < 50:
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
Kc_out = 0.4 # ~= (maxTheta-midTheta)/(midPos-maxPos)
KI_out = 0.0
KD_out = 1.5
# Previous: 5.0, 12
# Optimization showed that KD_out ~= 2*Kc_out
dt = 0.1 # seconds, measured using the closed-loop code
pidOuter = pc.PID(Kc_out, KI_out, KD_out, dt=dt, \
             outputLimits=[minAngle, maxAngle], ubias = zero)
# Inner PID configuration (angle error -> motor speed)
Kc_in = -3 # (maxTheta-midTheta)/-1
KI_in = 0.0
KD_in = -0.5
pidInner = pc.PID(Kc_in, KI_in, KD_in, dt = dt, \
                  outputLimits=[-1, 1], ubias = 0.0)
startTime = time.time() # seconds
reading = 0
thetaSPlast = midAngle
thetaLast = midAngle
currentSpeed = 0
print("Mode: manual")
try:
    while True:
        if mode == 'manual':
            stick = yButton.read()-yButtonMid
                
            # Scale stick input to the range on each side (asymmetrical)
            if stick < 0:
                stick = stick/(yButtonMid-yButtonMin)
            if stick > 0:
                stick = stick/(yButtonMax-yButtonMid)
                
            # Get rid of noise
            if abs(stick) < 0.4:
                stick = 0
                
            # Trim reading to binary
            reading = max(min(0.5, stick), -0.5)
            #print(f"stick: {stick:.4g}, reading: {reading:.4g}")
            
            # Actuate
            if boundsOK():
                motor.set_speed(reading)
            else:
                print("Bounds error! Correcting...")
                
            # Print data
            pos = position(output = False)
            thetaActual = mpu.read_angle()['y']
            newSpeed = reading
            print(f"pos: {pos:.4g}, posSP: {L/2:.4g}")
            
            # Detect click for switch to automatic mode
            if button.value() == 0 and (time.time()-startTime)%0.5<0.01:
                # The extra bit in the "and" statement reduces the
                # button's sensitivity by effectively only having it
                # read every half second.
                mode = 'auto'
                print("Mode: auto")
        elif mode == 'auto':
            alpha = 0.4
            SP = L/2/1000 # m
            pos = position()/1000 # m
            pidOuter.setpoint = SP
            thetaSP, Pout, Iout, Dout = pidOuter(pos)
            
            thetaSPlast = thetaSP*alpha + (1-alpha)*thetaSPlast
            pidInner.setpoint = thetaSPlast
            newTheta = mpu.read_angle()['y']
            # Gate filter theta measurements
            if abs(newTheta - thetaLast) > 0.5:
                newTheta = thetaLast
            # Alpha filter theta measurements
            thetaLast = newTheta*alpha + (1-alpha)*thetaLast
            newSpeed = pidInner(thetaLast)[0]
            currentSpeed = alpha*newSpeed + (1-alpha)*currentSpeed
            print(f"zero: {0.0}, L: {L/1000}, pos: {pos:.4g}, sp: {SP:.4g}")
            if boundsOK():
                motor.set_speed(newSpeed)
            else:
                pass
                #print("Bounds error! Correcting...")
            #print(f"pos: {pos:.4g}, posSP: {SP:.4g}")
            #print(f"thetaSP: {thetaSPlast:.4g}, theta: {thetaLast:.4g}, thetaMax: {maxAngle}, thetaMin: {minAngle}, zero: {zero:.4g}")
            if button.value() == 0 and (time.time()-startTime)%0.25<0.01:
                mode = 'manual'
                print("Mode: manual")
        else:
            print("Unknown error...")
except KeyboardInterrupt:
    print("Loop ended.")
    motor.set_speed(0)