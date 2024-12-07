import time
from hcsr04 import HCSR04

# Initialize Sensors
sensor1 = HCSR04(trigger_pin=16, echo_pin=15)
sensor2 = HCSR04(trigger_pin=14, echo_pin=13)

# Storage arrays
imax = 500
sensor1record = [0,]*imax
sensor2record = [0,]*imax

i = 0
print("Distances (mm)")
print("--------------")
while True and i <= imax:
    try:
        i += 1
        sensor1record[i] = sensor1.distance_mm()
        time.sleep(0.05)
        sensor2record[i] = sensor2.distance_mm()
        print(f'{sensor1record[i]:.4g}, {sensor2record[i]:.4g},')
        #print(f"{sensor1record[i]:.4g}, ")
        time.sleep(0.05)
    except KeyboardInterrupt:
        # Use Ctrl-C to end the test early
        print("Test ended.")
        break