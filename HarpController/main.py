# This is a sample Python script.

import serial
import time
import csv
import os
import math
import numpy as np
from scipy.interpolate import interp1d
from MuscleClass import Muscle

global ser, prevTime, CurrentTime, muscle, pressures_lookup, strains_lookup, maxP

############################
#Parameters
###########################
ComPort = 'COM4'
output_file = 'ControllerData/TrajectoryTracking_1hz_v3.csv'
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

buffer = []
dtBuffer = []
def parse_message(message):
    # Decode the byte message to a string
    decoded_message = message.decode('utf-8').strip()  # Remove any trailing newlines or carriage returns

    # Split the string into individual float strings
    float_strings = decoded_message.split(',')

    # Convert the strings to floats and append to a list
    float_list = [float(num) for num in float_strings]

    return float_list

# writes data stored in buffer to excel sheet
def write_to_csv_periodic(filename, buffer):
    """Write buffered data to a CSV file and clear buffer."""
    with open(filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(buffer)
    buffer.clear()  # Clear buffer after writing


def packAndSendMsg(P1):
    #Packs together our message, taking the command character and the text entries and sends it over serial
    global ser

    msg = 'A'+ ',' + str(P1) # Build Message
    msg = msg + 'Z'  # add end of message indicator
    ser.write(bytes(str(msg), 'UTF-8'))

def pressure_from_strain( strain_input):
    """
    Interpolates to find the pressure (in psi) corresponding to a given strain.
    """

    global pressures_lookup, strains_lookup


    # Interpolation: strain → pressure
    interpolator = interp1d(strains_lookup, pressures_lookup, kind='linear', fill_value=(0,40), bounds_error=False)
    pressure_estimate = float(interpolator(strain_input))

    return pressure_estimate


global L0, prevError, error, error_dot, errorIntegral, desiredDisp, disp, StartTime
L0 = 0
prevError = 0
error = 0
error_dot = 0
errorIntegral = 0
disp = 0
desiredDisp = 0

def getControlInput(dist):
    global prevTime, CurrentTime, L0, prevError, error, error_dot, errorIntegral, desiredDisp, disp, StartTime, muscle,maxP

    freq = 1
    # desiredStrain=.4
    desiredStrain = .3+.15*np.cos(2*np.pi*(CurrentTime-StartTime-3)*freq)

    desiredDisp = desiredStrain*(muscle.l0*1000)

    #sinusoidal Gains
    kp = .75
    kd = .001
    ki=.7
    kf1 = 1
    kf2 = .9
    offset = .0

    #.4
    # kp = .001
    # kd = .0
    # ki = .9
    # kf1 = 1
    # kf2 = 1
    # offset = .028*desiredStrain/.4


    # .1
    # kp = .1
    # kd = .01
    # ki = 10
    # kf1 = 1
    # kf2 = 1
    # offset = .0

    # # .2
    # kp = .001
    # kd = .01
    # ki = 5
    # kf1 = 1
    # kf2 = 1
    # offset = .0

    # .3
    # kp = .001
    # kd = .01
    # ki = 1.5
    # kf1 = 1
    # kf2 = 1
    # offset = .0


    dt = CurrentTime-prevTime

    if CurrentTime-StartTime<3:
        P = 0
        errorIntegral=0
        L0 = dist # find zero position
        error = desiredDisp - disp

    disp = dist - L0

    if CurrentTime-StartTime>=3:

        prevError = error

        ###############################
        #bASIC PID
        ###############################
        error = desiredDisp-disp
        error_dot = (prevError-error)/dt
        errorIntegral = errorIntegral + dt * (1 / 2) * (prevError + error)  # using trapezoidal integration

        u = error*kp + error_dot*kd +  errorIntegral*ki

        # desiredStrain = desiredDisp/(muscle.l0*1000)

        # P = maxP

        P=kf1*pressure_from_strain(kf2*desiredStrain+u/(muscle.l0*1000)-offset)
        # P = pressure_from_strain(desiredStrain)
        # P = (35/5)*(CurrentTime-StartTime -3)
        print("Actual Strain: " + str(disp/(muscle.l0*1000)) +" Error:"+ str(error) + "  DesiredStrain:"+str(desiredStrain) +
            "   P:"+str(P))
    ##########################
    #Saftey limits for P
    ##########################

    if P<0:
        P = 0
    if P>maxP:
        P = maxP

    return P


################################################
#Serial Communications setup
###############################################


ser = serial.Serial(port= ComPort, baudrate=115200, timeout=.1)  # create Serial Object, baud = 9600, read times out after 10s
# time.sleep(.1)  # delay 3 seconds to allow serial com to get established
print("Connected")

################################################
#Start control / data collection loop
###############################################
StartTime =time.time_ns()/(10**9)
prevTime = StartTime #start clock in s

while True:
    msg = ser.readline() #read serial line til \n

    if len(msg)>0:

        try:
            parsed_msg= parse_message(msg) # parse message in form b'float,float,float\r\n'
            # print(msg)
            if parsed_msg[-2] >= 300 or parsed_msg[-2] < 50:
                print("Error: Laser Sensor out of bounds!!!!")


            if not parsed_msg[-1]: #meaning the controller is waiting for an input
                CurrentTime = time.time_ns()/(10**9)  #time since controller started (global used in controller)
                P = getControlInput(parsed_msg[-2])
                packAndSendMsg(P)

            # check if enough time has passed for us to store a message
            if (time.time_ns()/(10**9) - prevTime) > 1/ReadFrequency:

                buffer.append(parsed_msg +[disp,desiredDisp]) # store message in buffer plus controller stuff
                dtBuffer.append(time.time_ns()/(10**9) - prevTime)

                prevTime = time.time_ns()/(10**9)

                if len(buffer) >= BATCH_SIZE: # when buffer is filled, write messages to the file and clear buffer
                    write_to_csv_periodic(output_file, buffer)

                    meanFreq = 1/(sum(dtBuffer) / len(dtBuffer))
                    dtBuffer=[]

                    print(f"Batch written. Time:{parsed_msg[0]}, Avg Freq:{meanFreq}")

        except UnicodeDecodeError:
            print("UnicodeDecodeError")

        except ValueError:
            print("Invalid input. Please enter a number.")