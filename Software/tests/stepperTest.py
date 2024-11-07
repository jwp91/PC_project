from machine import Pin, PWM
import time

# Pins on the arduino
in1 = 34
in2 = 35
in3 = 36
in4 = 37

# careful lowering this, at some point you run into the mechanical limitation of how quick your motor can move
step_sleep = 0.004

step_count = 4096 # 5.625*(1/64) per step, 4096 steps is 360°

direction = False # True for clockwise, False for counter-clockwise

# defining stepper motor sequence (found in documentation http://www.4tronix.co.uk/arduino/Stepper-Motors.php)
step_sequence = [[1,0,0,1],
                 [1,0,0,0],
                 [1,1,0,0],
                 [0,1,0,0],
                 [0,1,1,0],
                 [0,0,1,0],
                 [0,0,1,1],
                 [0,0,0,1]]

# setting up
p1 = Pin(in1, Pin.OUT)
p2 = Pin(in2, Pin.OUT)
p3 = Pin(in3, Pin.OUT)
p4 = Pin(in4, Pin.OUT)

# initializing
p1.value(0)
p2.value(0)
p3.value(0)
p4.value(0)

motor_pins = [p1, p2, p3, p4]
motor_step_counter = 0 ;

def cleanup():
    p1.value(0)
    p2.value(0)
    p3.value(0)
    p4.value(0)

# the meat
try:
    i = 0
    for i in range(step_count):
        for pinNum in range(len(motor_pins)):
            motor_pins[pinNum].value(step_sequence[motor_step_counter][pinNum])
        if direction==True:
            motor_step_counter = (motor_step_counter - 4) % 8
        elif direction==False:
            motor_step_counter = (motor_step_counter + 4) % 8
        else: # defensive programming
            print( "uh oh... direction should *always* be either True or False" )
            cleanup()
            exit( 1 )
        time.sleep( step_sleep )

except KeyboardInterrupt:
    cleanup()

cleanup()