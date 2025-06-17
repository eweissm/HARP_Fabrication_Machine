import os
from serial_communicator import SerialCommunicator
import time
import math

output_file= "Data/Muscle43_v1_40psi_1000g.csv"

if os.path.exists(output_file):
    raise FileExistsError(f"Error: The file '{output_file}' already exists. Choose a different name.")


communicator = SerialCommunicator(port="COM4",
                                  baudrate=115200,
                                  n_floats_to_arduino=1,
                                  n_floats_from_arduino=4,
                                  verbose=False,
                                  buffer_size=100,
                                  direction = 'TwoWay',
                                  logFreq=True,
                                  CSVPath = output_file,
                                  DesiredFreq = 500)
communicator.start()
prevTime = time.time()
startTime = prevTime
freq = 100

rampTime = 15
edgeTime = 3
cycle_duration = (edgeTime+rampTime)*2
maxP = 40

while True:

    now = time.time()
    dt = now- prevTime


    if dt >= 1/freq:
        prevTime = now

        t = now - startTime

        cycle_time = t % cycle_duration
        numCycles = math.floor(t/cycle_duration)
        if cycle_time < edgeTime:
            pressure = 0
        elif cycle_time < edgeTime + rampTime:
            pressure = (cycle_time - edgeTime) * (maxP / rampTime)
        elif cycle_time < edgeTime + rampTime + edgeTime:
            pressure = maxP
        elif cycle_time < edgeTime + rampTime + edgeTime + rampTime:
            pressure = maxP - (cycle_time - (edgeTime+rampTime+edgeTime)) * (maxP / rampTime)


        if cycle_time %1 < .01:
            print(numCycles)

        setP = [pressure]

        communicator.set_data_to_send(setP)
        # buffer = communicator.get_buffer()
        # print(buffer)
        # if buffer:
            # if buffer[-1][-1]> 350 or buffer[-1][-1]<50:
            #      print("Error:check laser limits")
            # print(buffer[-1][2])    else:
        time.sleep(0.001)

