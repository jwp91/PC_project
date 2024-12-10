# Joystick test

from machine import ADC,Pin
import time

pinVrx = 5
pinVry = 6
pinSW  = 7

xAxis = ADC(Pin(pinVrx, Pin.IN)) # create an ADC object acting on a pin
xAxis.atten(xAxis.ATTN_11DB)
yAxis = ADC(Pin(pinVry, Pin.IN)) # create an ADC object acting on a pin
yAxis.atten(yAxis.ATTN_11DB)
button = Pin(pinSW, Pin.IN, Pin.PULL_UP)

try: 
    while True:
        xValue = xAxis.read()  # read a raw analog value in the range 0-4095
        yValue = yAxis.read()  # read a raw analog value in the range 0-4095
        btnValue = button.value()
        print(f"X:{xValue}, Y:{yValue}, Button:{btnValue}")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Test ended.")