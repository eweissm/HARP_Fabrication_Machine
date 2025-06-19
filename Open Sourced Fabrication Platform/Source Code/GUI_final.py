"""
================================================================================
HARP Machine Controller GUI
--------------------------------------------------------------------------------
Author: Eric Weissman
Created: 6/16/2025
Description:
    This script provides a graphical user interface (GUI) for controlling a 
    PTCA (Percutaneous Transluminal Coronary Angioplasty) machine using Python 
    and serial communication with an Arduino-based hardware system.

    Users can input parameters for tube forming (twist angle, tube length, 
    diameters, FR pitch, etc.) and send commands such as:
        - Home
        - Cold Draw
        - Fiber Reinforce
        - Twist
        - Coil
        - Jog motions
        - Spin
        - Stop

    The GUI supports live COM port selection and communicates with the machine
    via formatted serial messages.

Dependencies:
    - Python 3.x
    - tkinter (standard GUI library)
    - pyserial (install via pip: `pip install pyserial`)

Usage:
    1. Run this script.
    2. Select the COM port and connect.
    3. Enter your desired tube and muscle properties.
    4. Click a control button to send a command to the hardware.

Communication Protocol:
    - Messages sent over serial are formatted as:
        <Command>,<Twist>,<Length>,<Tube Ø>,<Draw Ratio>,<FR Pitch>,<Coil Pitch>,<Mandrel Ø>Z
    - The Arduino listens for these messages and executes the corresponding action.

================================================================================
"""


import tkinter as tk       # GUI library
import serial              # Serial communication library
import time                # Timing utility

# Global variable declarations
global ser
global ConnectionStateMessage
global arduinoQueue
global localQueue
global ConnectionState
global SerialReq

################################################################################################
# Utility Functions
################################################################################################

def ReadInputs():
    """Reads values from the GUI entry widgets and returns them as a tuple."""
    twistAngle = TwistAngle_entry.get()
    tubeLength = TubeLength_entry.get()
    tubeDiameter = TubeDiameter_entry.get()
    coldDrawRatio = coldDrawRatio_entry.get()
    FRAngle = FRAngle_entry.get()
    coilAngle = CoilAngle_entry.get()
    MandrelDiameter = MandrelDiameter_entry.get()
    return (twistAngle, tubeLength, tubeDiameter, coldDrawRatio, FRAngle, coilAngle, MandrelDiameter)

def packAndSendMsg(Command):
    """Constructs a command string from the GUI inputs and sends it over the serial port."""
    global ser
    print(Command)
    Parameters = ReadInputs()
    msg = Command  # start with command character

    for i in Parameters:
        msg = msg + ',' + i  # append each parameter

    msg = msg + 'Z'  # terminate message with 'Z'

    ser.write(bytes(str(msg), 'UTF-8'))  # send message

# Placeholder for serial listener using queues (currently commented out)
# This could be useful for receiving data from Arduino asynchronously
# import queue
# arduinoQueue = queue.Queue()
# localQueue = queue.Queue()

ConnectionState = False  # Initialize connection flag

def ConnectSerial(PortString):
    """Attempts to connect to the Arduino on the specified COM port."""
    global ser
    global ConnectionStateMessage
    global ConnectionState

    ConnectionStateMessage.set("...")  # show connecting message
    try:
        ser = serial.Serial(port=PortString, baudrate=9600, timeout=10)
        time.sleep(0.1)  # slight delay to stabilize connection
        ConnectionStateMessage.set("Connected")
        ConnectionState = True
    except:
        ConnectionStateMessage.set("Connection Failed")
        ConnectionState = False

def runStartUp():
    """Launches a startup window to prompt the user for a COM port."""
    global ConnectionState
    global ConnectionStateMessage

    tkTop = tk.Tk()
    tkTop.geometry('600x200')
    tkTop.title("PTCA-Machine Startup")

    ConnectionStateMessage = tk.StringVar()
    ConnectionStateMessage.set("Not Connected")

    # Layout: COM port input section
    Title = tk.Label(text='Please Select a Com Port', font=("Courier", 14, 'bold')).pack()
    BodyFrame = tk.Frame(master=tkTop)
    BodyFrame.pack(fill=tk.BOTH, side=tk.TOP, expand=True)

    tk.Label(master=BodyFrame, text='Com Port:', font=("Courier", 12, 'bold')).pack(side='left')
    ComPortInput_entry = tk.Entry(BodyFrame)
    ComPortInput_entry.insert(0, 'com4')
    ComPortInput_entry.pack(side='left')
    tk.Label(master=BodyFrame, text='(Example: com3)', font=("Courier", 12, 'bold')).pack(side='left')

    # Layout: Connect and Exit buttons
    ButtonFrame = tk.Frame(master=tkTop)
    ButtonFrame.pack(fill=tk.BOTH, side=tk.TOP, expand=True)

    ConnectButton = tk.Button(ButtonFrame, text="Connect",
                              command=lambda: ConnectSerial(ComPortInput_entry.get()),
                              height=3, fg="black", width=10, bd=5, activebackground='green')
    ConnectButton.pack(side='left', ipadx=10, padx=10, pady=40)

    tk.Label(master=ButtonFrame, textvariable=ConnectionStateMessage,
             font=("Courier", 10)).pack(side='left')

    ExitStartup_Button = tk.Button(ButtonFrame, text="Done", command=tkTop.destroy,
                                   height=4, fg="black", width=10, bd=5, activebackground='green')
    ExitStartup_Button.pack(side='right', ipadx=10, padx=10, pady=40)

    SerialReq = tk.IntVar()
    checkbutton = tk.Checkbutton(master=ButtonFrame, text="Disable Serial Req",
                                 variable=SerialReq, onvalue=True, offvalue=False)
    checkbutton.pack(side='right', ipadx=10, padx=10, pady=40)

    tk.mainloop()
    return ConnectionState, SerialReq.get()

################################################################################################
# Establish Serial Connection
################################################################################################

ComsState = False
SerialReq = 0

# Loop until either connection succeeds or user disables requirement
while not ComsState and SerialReq == 0:
    ComsState, SerialReq = runStartUp()

################################################################################################
# Main Controller GUI
################################################################################################

tkTop = tk.Tk()
tkTop.geometry('600x600')
tkTop.title("PTCA-Machine Controller")

tk.Label(text='PTCA Machine Controller', font=("Courier", 14, 'bold')).pack()

###############################
# Tube Properties Section
###############################

TubePropertiesFrame = tk.Frame(master=tkTop)
TubePropertiesFrame.pack(fill=tk.BOTH, side=tk.TOP, expand=True)

tk.Label(master=TubePropertiesFrame, text='Tube Properties',
         font=("Courier", 12, 'bold')).pack()

# Individual input fields
for label, default, var in [
    ("Tube Length (mm)", '220', 'TubeLength_entry'),
    ("Tube Diameter (mm)", '2.5', 'TubeDiameter_entry'),
    ("Mandrel Diameter (mm)", '3.0', 'MandrelDiameter_entry')
]:
    frame = tk.Frame(master=TubePropertiesFrame)
    frame.pack(fill=tk.BOTH, side=tk.TOP, expand=True)
    tk.Label(master=frame, text=label, font=("Courier", 12, 'bold')).pack(side='left', padx=10)
    globals()[var] = tk.Entry(frame)
    globals()[var].insert(0, default)
    globals()[var].pack(side='left')

###############################
# Muscle Properties Section
###############################

MusclePropertiesFrame = tk.Frame(master=tkTop)
MusclePropertiesFrame.pack(fill=tk.BOTH, side=tk.TOP, expand=True)

tk.Label(master=MusclePropertiesFrame, text='Muscle Properties',
         font=("Courier", 12, 'bold')).pack()

# Individual muscle properties fields
for label, default, var in [
    ("Twist Angle (Deg)", '35', 'TwistAngle_entry'),
    ("FR Pitch (mm)", '12', 'FRAngle_entry'),
    ("Cold Draw Ratio", '2.5', 'coldDrawRatio_entry'),
    ("Coil Pitch (mm)", '6', 'CoilAngle_entry')
]:
    frame = tk.Frame(master=MusclePropertiesFrame)
    frame.pack(fill=tk.BOTH, side=tk.TOP, expand=True)
    tk.Label(master=frame, text=label, font=("Courier", 12, 'bold')).pack(side='left', padx=10)
    globals()[var] = tk.Entry(frame)
    globals()[var].insert(0, default)
    globals()[var].pack(side='left')

###############################
# Control Buttons
###############################

ControlsFrame = tk.Frame(master=tkTop)
ControlsFrame.pack(fill=tk.BOTH, side=tk.TOP, expand=True)

tk.Label(master=ControlsFrame, text='Controls',
         font=("Courier", 12, 'bold')).pack()

# Control button setup helper
def make_control_button(master, text, cmd):
    return tk.Button(master, text=text, command=lambda: packAndSendMsg(cmd),
                     height=3, fg="black", width=10, bd=5, activebackground='green')

# Top row of control buttons
TopRow = tk.Frame(master=ControlsFrame); TopRow.pack()
for label, cmd in [
    ("Home", 'A'), ("Cold Draw", 'B'), ("Fiber Reinforce", 'C'),
    ("Twist", 'D'), ("Coil", 'E')
]:
    make_control_button(TopRow, label, cmd).pack(side='left', padx=10, pady=10)

# Middle row (jogging controls)
MiddleRow = tk.Frame(master=ControlsFrame); MiddleRow.pack()
for label, cmd in [
    ("Follower Up", 'G'), ("Follower Down", 'H'),
    ("Runner Up", 'I'), ("Runner Down", 'J'),
    ("Spin", 'K')
]:
    make_control_button(MiddleRow, label, cmd).pack(side='left', padx=10, pady=10)

# Bottom row (stop button)
BottomRow = tk.Frame(master=ControlsFrame); BottomRow.pack()
tk.Button(BottomRow, text="Stop", command=lambda: packAndSendMsg('F'),
          height=3, fg="black", width=10, bd=5, activebackground='red').pack(pady=10)

tk.mainloop()
