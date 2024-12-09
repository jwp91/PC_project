# Example code for (GY-521) MPU6050 Accelerometer/Gyro Module
# Write in MicroPython by Warayut Poomiwatracanont JAN 2023

from MPU6050 import MPU6050

from os import listdir, chdir
from machine import Pin
from time import sleep_ms

mpu = MPU6050(sclPin = 37, sdaPin = 38)

# List all files directory.
print("Root directory: {} \n".format(listdir()))

# Change filename and fileformat here.
filename = "{}%s.{}".format("data_logs", "txt")

# Increment the filename number if the file already exists.
i = 0
while (filename % i) in listdir():
    i += 1

# Save file in path /
with open(filename % i, "w") as f:
    cols = ["Temp", "AcX", "AcY", "AcZ", "gX", "gY", "gX"]
    f.write(",".join(cols) + "\n")
    
    try:
        while True:
            # Accelerometer Data
            accel = mpu.read_accel_data() # read the accelerometer [ms^-2]
            aX = accel["x"]
            aY = accel["y"]
            aZ = accel["z"]
            #print(f"Accelerometer | x: {aX} y: {aY} z: {aZ}")
        
            #Gyroscope Data
            gyro = mpu.read_gyro_data()   # read the gyro [deg/s]
            gX = gyro["x"]
            gY = gyro["y"]
            gZ = gyro["z"]
            #print(f"Gyroscope     | x: {gX} y: {gY} z: {gZ}")
        
            # Rough Temperature
            temp = mpu.read_temperature()   # read the device temperature [degC]
            #print(f"Temperature: {temp} °C")

            # G-Force
            # gforce = mpu.read_accel_abs(g=True) # read the absolute acceleration magnitude
            #print("G-Force: " + str(gforce))
            
            # Angle
            angle = mpu.read_angle()
            print(f"Angle: x = {angle['x']}, y = {angle['y']}")
        
            # Write to file
            data = {"Temp" : temp,
                    "AcX" : aX,
                    "AcY" : aY,
                    "AcZ" : aZ,
                    "gX"  : gX,
                    "gY"  : gY,
                    "gZ"  : gZ
                    }
            
            push = [  str(data[k]) for k in cols ]
            row = ",".join(push)
            f.write(row + "\n")
            
            # Time Interval Delay in millisecond (ms)
            sleep_ms(100)
    except KeyboardInterrupt:
        print("Test ended.")
