# Class for controlling a continuous-type servo motor
from machine import Pin, PWM
from time import sleep

class servo:
    def __init__(self, pin, min_speed = 0.2):
        """
Initializes the continuous servo.
Inputs:
    pin = GPIO pin on ESP32
    min_speed = minimum pulse (between 0 and 1) that the motor can actuate.
        Ex. In the case of MG90s motors, this is ~0.089 = 8.9% of "full power".
        This could be tested manually by setting min_speed=0 and then running a series of
        tests to determine at what X set_speed(X) results in a stopped motor.
"""
        self.pin = Pin(pin, Pin.OUT)    # Set pin for GPIO interfacing
        self.min_speed = min_speed
        self.motor = PWM(self.pin, freq=50)  # Set frequency to 50Hz
        self.set_speed(0)
        # NOTE: MG90s has a 20 ms period (source: ChatGPT), which corresponds to
        # a 50 Hz refresh speed.
        
    def set_speed(self, speed):
        """
Sets the servo direction.
Inputs:
    speed: float := (-1,1).
        -1 corresponds to counterclockwise rotation
         1 corresponds to clockwise rotation.
         0 corresponds to stopped
         Any float inside this range is acceptable.
Output:
    None, sets motor rotation.
"""
        if speed < -1 or speed > 1:
            raise ValueError("Speed parameters must be between -1 and 1.")
        
        # Account for the motor not being able to do a speed lower than speed = self.min_speed
        if speed < 0:
            speed = (1-self.min_speed)*speed - self.min_speed
        elif speed != 0:
            speed = (1-self.min_speed)*speed + self.min_speed
        else:
            pass #speed = 0 stays the same
        
        # Convert to duty and write to motor
        pwParam = speed*500 + 1500            # Pulse width parameter
        duty = int((pwParam / 20000) * 1023)  # Scale 20ms period to 10-bit duty cycle
        self.motor.duty(duty)                 # Write motor duty
        return None
    
    def test(self):
        print("Rotating counterclockwise at full speed")
        self.set_speed(-1)    # Full speed counterclockwise
        sleep(2)
        
        print("Rotating counterclockwise at medium speed")
        self.set_speed(-0.5)  # Medium speed counterclockwise
        sleep(2)
        
        print("Stopping the servo")
        self.set_speed(0)     # Stop
        sleep(2)
        
        print("Rotating clockwise at medium speed")
        self.set_speed(0.5)  # Medium speed clockwise
        sleep(2)
        
        print("Rotating clockwise at full speed")
        self.set_speed(1)    # Full speed clockwise
        sleep(2)
        
        print("Stopping the servo")
        self.set_speed(0)    # Stop
        
        print("Test finished")
        return None