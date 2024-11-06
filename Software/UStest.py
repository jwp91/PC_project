import time
from hcsr04 import HCSR04

sensor = HCSR04(trigger_pin=16, echo_pin=0)

imax = 250
i = 0
while True and i <= imax:
    try:
        i += 1
        print(f'Distance (mm): {sensor.distance_mm()}')
        time.sleep(0.5)
    except KeyboardInterrupt:
        # Use Ctrl-C to end the test early
        print("Test ended.")
        break