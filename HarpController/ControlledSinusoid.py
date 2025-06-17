
import serial
import time
import csv
import os
import math
import numpy as np
from scipy.interpolate import interp1d
from MuscleClass import Muscle
from serial_communicator import SerialCommunicator

global muscle, pressures_lookup, strains_lookup, maxP
global L0, prevError, error, error_dot, errorIntegral, desiredDisp, disp

############################
#Parameters
###########################
output_file = 'ControllerData/TrajectoryTracking_3hz_v3.csv'
BATCH_SIZE = 1000 # Write to file every 10 entries
ReadFrequency = 1000 #hz -- 50 for fatigue, 100 for isotonic, 1000 for response time
maxP=50

ConfigFile= "MuscleConfig.ini"

#######################
#Build Muscle Model from muscle class. Save the lookup table
#######################
muscle = Muscle(ConfigFile)
lookup_table = muscle.strain_lookup(0,maxP,.1, .02*9.81)
strains_lookup = np.array(list(lookup_table.keys()))
pressures_lookup = np.array(list(lookup_table.values()))


# throw error if file already exists
if os.path.exists(output_file):
    raise FileExistsError(f"Error: The file '{output_file}' already exists. Choose a different name.")


################################################
#Serial Communications setup
###############################################
communicator = SerialCommunicator(port="COM4",
                                  baudrate=115200,
                                  n_floats_to_arduino=1,
                                  n_floats_from_arduino=4,
                                  verbose=False,
                                  buffer_size=100,
                                  direction = 'TwoWay',
                                  logFreq=True,
                                  DesiredFreq = 500)
communicator.start()

CSVbuffer = []

# writes data stored in buffer to excel sheet
def write_to_csv_periodic(filename, CSVbuffer):
    """Write buffered data to a CSV file and clear buffer."""
    with open(filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(CSVbuffer)
    CSVbuffer.clear()  # Clear buffer after writing


def pressure_from_strain( strain_input):
    """
    Interpolates to find the pressure (in psi) corresponding to a given strain.
    """

    global pressures_lookup, strains_lookup


    # Interpolation: strain → pressure
    interpolator = interp1d(strains_lookup, pressures_lookup, kind='linear', fill_value=(0,40), bounds_error=False)
    pressure_estimate = float(interpolator(strain_input))

    return pressure_estimate

L0 = 0
prevError = 0
error = 0
error_dot = 0
errorIntegral = 0
disp = 0
desiredDisp = 0

def getControlInput(dist, t, dt):
    global L0, prevError, error, error_dot, errorIntegral, desiredDisp, disp, muscle,maxP

    freq = 3

    desiredStrain = .3+.15*np.cos(2*np.pi*(t-3)*freq)

    desiredDisp = desiredStrain*(muscle.l0*1000)

    #1hz sinusoidal Gains
    # kp = .7
    # kd = .01
    # ki=2
    # kf1 = 1
    # kf2 = .9
    # offset = .0

    # # 2hz sinusoidal Gains
    # kp = .5
    # kd = .015
    # ki = 2.5
    # kf1 = 1
    # kf2 = .9
    # offset = .0

    # 3hz sinusoidal Gains
    kp = .5
    kd = .001
    ki = 3
    kf1 = 1
    kf2 = .9
    offset = .0

    disp = dist - L0

    if t<3:
        P = 0
        errorIntegral=0
        L0 = dist # find zero position
        error = desiredDisp - disp
    else:

        prevError = error

        ###############################
        #bASIC PID
        ###############################
        error = desiredDisp-disp
        error_dot = (prevError-error)/dt
        errorIntegral = errorIntegral + dt * (1 / 2) * (prevError + error)  # using trapezoidal integration

        u = error*kp + error_dot*kd +  errorIntegral*ki


        P=kf1*pressure_from_strain(kf2*desiredStrain+u/(muscle.l0*1000)-offset)

        # print("Actual Strain: " + str(disp/(muscle.l0*1000)) +" Error:"+ str(error) + "  DesiredStrain:"+str(desiredStrain) +
        #     "   P:"+str(P))
    ##########################
    #Saftey limits for P
    ##########################

    if P<0:
        P = 0
    if P>maxP:
        P = maxP

    return P


MainLoop_prevTime = time.time()
MainLoop_startTime = MainLoop_prevTime
freq = 100

while True:

    now = time.time()
    dt = now- MainLoop_prevTime


    if dt >= 1/freq:
        MainLoop_prevTime = now
        t = now - MainLoop_startTime

        buffer = communicator.get_buffer()

        if buffer:
            pressure= getControlInput(buffer[-1][-1], t, dt)

            communicator.set_data_to_send([pressure])

            CSVbuffer.append(buffer[-1] +[disp,desiredDisp])

        if len(CSVbuffer) >= BATCH_SIZE:  # when buffer is filled, write messages to the file and clear buffer
            write_to_csv_periodic(output_file, CSVbuffer)

    else:
        time.sleep(0.001)
